#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <ff.h>

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 4
#include <esp32/rom/crc.h>
#else
#include <rom/crc.h>
#endif

#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "usbxfer.h"
#include "sdcard.h"
#include "input.h"

#ifndef PROJECT_VER
    #define PROJECT_VER "n/a"
#endif

#ifdef TARGET_MRGC_G32
#define TARGET_NAME  "mrgc-g32"
#define FIRMWARE_DIR "/espgbc/firmware"
#else
#define TARGET_NAME  "odroid-go"
#define FIRMWARE_DIR "/odroid/firmware"
#endif

// See tools/sdxfer.py for the host side of the protocol.
//
// Frame: [0xA5 0x5A][cmd u8][seq u16][len u16][payload <len>][crc32 u32], little endian.
// The CRC covers cmd, seq, len and payload. Replies use (cmd | CMD_REPLY) with the request's seq
// and a payload starting with a status byte (0 = OK, 1..19 = FatFs FRESULT, 100+ = ST_*).
// File data is sent in CMD_DATA frames numbered from 0, acknowledged with go-back-N
// (CMD_ACK = received up to seq, CMD_NAK = resend from seq), XFER_WINDOW frames in flight.

#define XFER_UART           UART_NUM_0
#define XFER_PROTO_VERSION  1
#define XFER_DEFAULT_BAUD   115200
#define XFER_MAX_BAUD       2000000
#define XFER_RX_BUFFER      (32 * 1024)
#define XFER_CHUNK_SIZE     4096
#define XFER_MAX_PAYLOAD    (XFER_CHUNK_SIZE + 16)
#define XFER_WINDOW         4
#define XFER_HEADER_LEN     7
#define XFER_MAGIC0         0xA5
#define XFER_MAGIC1         0x5A
#define XFER_PATH_MAX       256
#define XFER_MAX_APPS       32

#define BAUD_PROBATION_MS   2000    // Revert to the default baud if the host goes silent after a switch
#define HOST_IDLE_MS        10000   // End a host-started session if the host goes silent
#define TRANSFER_IDLE_MS    10000   // Abort a file transfer if the host goes silent
#define DRAW_INTERVAL_MS    500

enum
{
    CMD_HELLO = 0x01,
    CMD_SET_BAUD,
    CMD_PING,
    CMD_BYE,
    CMD_STATFS = 0x10,
    CMD_LIST,
    CMD_STAT,
    CMD_MKDIR,
    CMD_RMDIR,
    CMD_RM,
    CMD_RENAME,
    CMD_GET = 0x20,
    CMD_GET_END,
    CMD_PUT,
    CMD_PUT_END,
    CMD_INSTALL = 0x28,
    CMD_APPS,
    CMD_ERASE,
    CMD_DATA = 0x30,
    CMD_ACK,
    CMD_NAK,
    CMD_REPLY = 0x80,
};

enum
{
    ST_OK = 0,
    ST_BAD_REQUEST = 100,
    ST_BAD_PATH,
    ST_CRC_MISMATCH,
    ST_SIZE_MISMATCH,
    ST_ABORTED,
    ST_INSTALL_FAILED,
    ST_TIMEOUT,
    ST_NOT_A_FILE,
    ST_NOT_A_DIR,
    ST_APP_ERROR,
};

typedef struct
{
    uint8_t cmd;
    uint16_t seq;
    uint16_t len;
    uint8_t data[XFER_MAX_PAYLOAD + 4];
} frame_t;

typedef struct
{
    const usbxfer_hooks_t *hooks;
    frame_t req;
    bool pending;           // req holds a frame that still needs to be handled by the main loop
    uint8_t tx[XFER_HEADER_LEN + XFER_MAX_PAYLOAD + 4];
    FIL file;
    FF_DIR dir;
    FILINFO info;
    char path[2][XFER_PATH_MAX + 16]; // "0:" + path + ".part"
    uint32_t baud;
    TickType_t baud_deadline;
    uint32_t buttons;
    TickType_t last_draw;
    char header[48];
} xfer_t;

#define TX_PAYLOAD (x->tx + XFER_HEADER_LEN)

static xfer_t *x;
static bool driver_installed = false;
static int magic_state = 0; // 0: scanning, 1: got MAGIC0, 2: got both (frame header follows)


static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void put_u32(uint8_t *p, uint32_t v) { put_u16(p, v); put_u16(p + 2, v >> 16); }
static inline void put_u64(uint8_t *p, uint64_t v) { put_u32(p, v); put_u32(p + 4, v >> 32); }
static inline uint16_t get_u16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t get_u32(const uint8_t *p) { return get_u16(p) | ((uint32_t)get_u16(p + 2) << 16); }
static inline uint64_t get_u64(const uint8_t *p) { return get_u32(p) | ((uint64_t)get_u32(p + 4) << 32); }

static inline bool time_reached(TickType_t deadline)
{
    return (int32_t)(xTaskGetTickCount() - deadline) >= 0;
}


void usbxfer_init(void)
{
    if (!driver_installed)
    {
        driver_installed = uart_driver_install(XFER_UART, XFER_RX_BUFFER, 0, 0, NULL, 0) == ESP_OK;
    }
}

bool usbxfer_host_detected(void)
{
    size_t available = 0;
    uint8_t byte;

    if (!driver_installed || uart_get_buffered_data_len(XFER_UART, &available) != ESP_OK)
        return false;

    while (available-- > 0 && uart_read_bytes(XFER_UART, &byte, 1, 0) == 1)
    {
        if (magic_state == 1 && byte == XFER_MAGIC1)
        {
            magic_state = 2;
            return true;
        }
        magic_state = (byte == XFER_MAGIC0);
    }

    return false;
}


// Returns 1 on success, 0 on timeout, -1 if a corrupt frame was received
static int recv_frame(frame_t *f, int timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    uint8_t header[XFER_HEADER_LEN - 2];
    uint8_t byte;

    while (magic_state != 2)
    {
        int32_t remaining = deadline - xTaskGetTickCount();
        if (remaining < 0 || uart_read_bytes(XFER_UART, &byte, 1, remaining) != 1)
            return 0;
        if (magic_state == 1 && byte == XFER_MAGIC1)
            magic_state = 2;
        else
            magic_state = (byte == XFER_MAGIC0);
    }
    magic_state = 0;

    // The rest of the frame must follow promptly
    if (uart_read_bytes(XFER_UART, header, sizeof(header), pdMS_TO_TICKS(200)) != sizeof(header))
        return -1;

    f->cmd = header[0];
    f->seq = get_u16(header + 1);
    f->len = get_u16(header + 3);

    if (f->len > XFER_MAX_PAYLOAD)
        return -1;

    if (uart_read_bytes(XFER_UART, f->data, f->len + 4, pdMS_TO_TICKS(1000)) != f->len + 4)
        return -1;

    uint32_t crc = crc32_le(0, header, sizeof(header));
    crc = crc32_le(crc, f->data, f->len);
    if (crc != get_u32(f->data + f->len))
        return -1;

    return 1;
}

// data can be NULL if the payload is already in place at TX_PAYLOAD
static void send_frame(uint8_t cmd, uint16_t seq, const void *data, size_t len)
{
    uint8_t *p = x->tx;

    p[0] = XFER_MAGIC0;
    p[1] = XFER_MAGIC1;
    p[2] = cmd;
    put_u16(p + 3, seq);
    put_u16(p + 5, len);
    if (data && len)
        memcpy(TX_PAYLOAD, data, len);
    put_u32(TX_PAYLOAD + len, crc32_le(0, p + 2, XFER_HEADER_LEN - 2 + len));

    uart_write_bytes(XFER_UART, (const char *)p, XFER_HEADER_LEN + len + 4);
}

// The reply's extra data (after the status byte) must already be at TX_PAYLOAD + 1
static void send_reply(const frame_t *req, uint8_t cmd, uint8_t status, size_t extra_len)
{
    TX_PAYLOAD[0] = status;
    send_frame(cmd | CMD_REPLY, req->seq, NULL, extra_len + 1);
}

static void send_status(const frame_t *req, uint8_t status)
{
    send_reply(req, req->cmd, status, 0);
}

// Error reply carrying a message for the user
static void send_error(const frame_t *req, uint8_t status, const char *message)
{
    size_t len = strlen(message);
    memcpy(TX_PAYLOAD + 1, message, len);
    send_reply(req, req->cmd, status, len);
}

static void send_ack(uint8_t cmd, uint16_t seq, uint8_t status)
{
    send_frame(cmd, seq, &status, 1);
}


static bool exit_requested(void)
{
    uint32_t state = input_get_state();
    bool pressed = (state & ~x->buttons) & (1 << ODROID_INPUT_B);
    x->buttons = state;
    return pressed;
}

static void draw_status(const char *message, int percent, bool force)
{
    if (!force && !time_reached(x->last_draw + pdMS_TO_TICKS(DRAW_INTERVAL_MS)))
        return;
    x->hooks->status(x->header, message, percent);
    x->last_draw = xTaskGetTickCount();
}

static void draw_progress(const char *path, uint64_t done, uint64_t total, TickType_t started, bool force)
{
    char message[64];
    const char *name = strrchr(path, '/') ? strrchr(path, '/') + 1 : path;
    uint32_t elapsed_ms = (xTaskGetTickCount() - started) * portTICK_PERIOD_MS;
    uint32_t speed = elapsed_ms ? (uint32_t)(done * 1000 / 1024 / elapsed_ms) : 0;

    snprintf(message, sizeof(message), "%.24s  %u KB/s", name, speed);
    draw_status(message, total ? (int)(done * 100 / total) : 100, force);
}


// Converts a card-relative path from a request ("/roms/nes/a.nes") to a FatFs path ("0:/roms/nes/a.nes")
static bool make_path(char *out, const uint8_t *src, size_t len)
{
    if (len == 0 || len > XFER_PATH_MAX || src[0] != '/')
        return false;

    // Only printable ASCII: the FatFs API encoding is set to the OEM code page
    for (size_t i = 0; i < len; i++)
    {
        if (src[i] < 0x20 || src[i] >= 0x7F)
            return false;
    }

    while (len > 1 && src[len - 1] == '/')
        len--;

    out[0] = '0' + odroid_sdcard_get_drive();
    out[1] = ':';
    memcpy(out + 2, src, len);
    out[2 + len] = 0;
    return true;
}

static bool is_root(const char *path)
{
    return strcmp(path + 2, "/") == 0;
}

static uint32_t fat_to_unix(WORD fdate, WORD ftime)
{
    if (fdate == 0)
        return 0;

    // FAT timestamps are local time, the host treats them the same way (TZ is UTC here)
    struct tm tm = {
        .tm_year = ((fdate >> 9) & 0x7F) + 80,
        .tm_mon = ((fdate >> 5) & 0x0F) - 1,
        .tm_mday = fdate & 0x1F,
        .tm_hour = (ftime >> 11) & 0x1F,
        .tm_min = (ftime >> 5) & 0x3F,
        .tm_sec = (ftime & 0x1F) * 2,
    };
    return mktime(&tm);
}


static void cmd_hello(const frame_t *req)
{
    uint8_t *p = TX_PAYLOAD + 1;

    // The host sends its local wall-clock time, so new files get sensible timestamps
    if (req->len >= 5 && get_u32(req->data + 1) != 0)
    {
        struct timeval tv = {.tv_sec = get_u32(req->data + 1), .tv_usec = 0};
        settimeofday(&tv, NULL);
    }

    *p++ = XFER_PROTO_VERSION;
    put_u32(p, XFER_MAX_BAUD), p += 4;
    put_u16(p, XFER_CHUNK_SIZE), p += 2;
    *p++ = XFER_WINDOW;
    p += sprintf((char *)p, "%s", TARGET_NAME) + 1;
    p += sprintf((char *)p, "%s", PROJECT_VER) + 1;
    p += sprintf((char *)p, "%s", FIRMWARE_DIR) + 1;

    send_reply(req, req->cmd, ST_OK, p - (TX_PAYLOAD + 1));

    snprintf(x->header, sizeof(x->header), "Connected (%u bps)", x->baud);
    draw_status("Waiting for commands...", -1, true);
}

static void cmd_set_baud(const frame_t *req)
{
    uint32_t baud = req->len >= 4 ? get_u32(req->data) : 0;

    if (baud < 9600 || baud > XFER_MAX_BAUD)
    {
        send_status(req, ST_BAD_REQUEST);
        return;
    }

    send_status(req, ST_OK);
    uart_wait_tx_done(XFER_UART, pdMS_TO_TICKS(100));
    uart_set_baudrate(XFER_UART, baud);
    x->baud = baud;
    x->baud_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(BAUD_PROBATION_MS);

    snprintf(x->header, sizeof(x->header), "Connected (%u bps)", x->baud);
    draw_status("Waiting for commands...", -1, true);
}

static void cmd_ping(const frame_t *req)
{
    // Optional payload: number of bytes to echo back, the host uses it to test the link at full frame size
    size_t len = req->len >= 2 ? get_u16(req->data) : 0;

    if (len > XFER_CHUNK_SIZE)
        len = XFER_CHUNK_SIZE;
    for (size_t i = 0; i < len; i++)
        TX_PAYLOAD[1 + i] = i * 151 + 13;

    send_reply(req, req->cmd, ST_OK, len);
}

static void cmd_statfs(const frame_t *req)
{
    char drive[3] = {'0' + odroid_sdcard_get_drive(), ':', 0};
    DWORD free_clusters;
    FATFS *fs;

    FRESULT res = f_getfree(drive, &free_clusters, &fs);
    if (res != FR_OK)
    {
        send_status(req, res);
        return;
    }

#if FF_MAX_SS != FF_MIN_SS
    uint64_t cluster_size = (uint64_t)fs->csize * fs->ssize;
#else
    uint64_t cluster_size = (uint64_t)fs->csize * FF_MAX_SS;
#endif
    put_u64(TX_PAYLOAD + 1, (uint64_t)(fs->n_fatent - 2) * cluster_size);
    put_u64(TX_PAYLOAD + 9, (uint64_t)free_clusters * cluster_size);
    send_reply(req, req->cmd, ST_OK, 16);
}

static void cmd_list(const frame_t *req)
{
    char *path = x->path[0];
    uint32_t start = req->len >= 4 ? get_u32(req->data) : 0;
    uint32_t index = 0;
    uint16_t count = 0;
    uint8_t more = 0;
    // Reply: status, more u8, count u16, entries [is_dir u8, size u64, mtime u32, name_len u8, name]
    uint8_t *p = TX_PAYLOAD + 4;
    uint8_t *end = TX_PAYLOAD + XFER_MAX_PAYLOAD;

    if (req->len < 5 || !make_path(path, req->data + 4, req->len - 4))
    {
        send_status(req, ST_BAD_PATH);
        return;
    }

    FRESULT res = f_opendir(&x->dir, path);
    if (res != FR_OK)
    {
        send_status(req, res);
        return;
    }

    while ((res = f_readdir(&x->dir, &x->info)) == FR_OK && x->info.fname[0])
    {
        const char *name = x->info.fname;
        size_t name_len = strlen(name);

        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0)
            continue;

        if (index++ < start)
            continue;

        if (p + 14 + name_len > end)
        {
            more = 1;
            break;
        }

        *p++ = (x->info.fattrib & AM_DIR) ? 1 : 0;
        put_u64(p, x->info.fsize), p += 8;
        put_u32(p, fat_to_unix(x->info.fdate, x->info.ftime)), p += 4;
        *p++ = name_len;
        memcpy(p, name, name_len), p += name_len;
        count++;
    }

    f_closedir(&x->dir);

    if (res != FR_OK)
    {
        send_status(req, res);
        return;
    }

    TX_PAYLOAD[1] = more;
    put_u16(TX_PAYLOAD + 2, count);
    send_reply(req, req->cmd, ST_OK, p - (TX_PAYLOAD + 1));
}

static void cmd_stat(const frame_t *req)
{
    char *path = x->path[0];

    if (!make_path(path, req->data, req->len))
    {
        send_status(req, ST_BAD_PATH);
        return;
    }

    // Reply: status, is_dir u8, size u64, mtime u32
    if (is_root(path))
    {
        memset(TX_PAYLOAD + 1, 0, 13);
        TX_PAYLOAD[1] = 1;
        send_reply(req, req->cmd, ST_OK, 13);
        return;
    }

    FRESULT res = f_stat(path, &x->info);
    if (res != FR_OK)
    {
        send_status(req, res);
        return;
    }

    TX_PAYLOAD[1] = (x->info.fattrib & AM_DIR) ? 1 : 0;
    put_u64(TX_PAYLOAD + 2, x->info.fsize);
    put_u32(TX_PAYLOAD + 10, fat_to_unix(x->info.fdate, x->info.ftime));
    send_reply(req, req->cmd, ST_OK, 13);
}

static void cmd_path_op(const frame_t *req)
{
    char *path = x->path[0];
    FRESULT res;

    if (!make_path(path, req->data, req->len) || is_root(path))
    {
        send_status(req, ST_BAD_PATH);
        return;
    }

    if (req->cmd == CMD_MKDIR)
    {
        send_status(req, f_mkdir(path));
        return;
    }

    // f_unlink removes both files and empty directories, check that we remove what was asked for
    if ((res = f_stat(path, &x->info)) != FR_OK)
    {
        send_status(req, res);
        return;
    }

    bool is_dir = x->info.fattrib & AM_DIR;
    if (req->cmd == CMD_RMDIR && !is_dir)
        send_status(req, ST_NOT_A_DIR);
    else if (req->cmd == CMD_RM && is_dir)
        send_status(req, ST_NOT_A_FILE);
    else
        send_status(req, f_unlink(path));
}

static void cmd_rename(const frame_t *req)
{
    // Payload: from \0 to
    const uint8_t *sep = memchr(req->data, 0, req->len);

    if (!sep
        || !make_path(x->path[0], req->data, sep - req->data)
        || !make_path(x->path[1], sep + 1, req->len - (sep + 1 - req->data))
        || is_root(x->path[0]) || is_root(x->path[1]))
    {
        send_status(req, ST_BAD_PATH);
        return;
    }

    send_status(req, f_rename(x->path[0], x->path[1]));
}

// Sends the open file as CMD_DATA frames (go-back-N). Returns a status code.
static uint8_t send_file(const char *path, uint64_t size, uint32_t *crc_out)
{
    uint32_t total = (size + XFER_CHUNK_SIZE - 1) / XFER_CHUNK_SIZE;
    uint32_t base = 0, next = 0, crc_next = 0;
    uint32_t crc = 0;
    int timeouts = 0;
    TickType_t started = xTaskGetTickCount();
    FRESULT res;

    while (base < total)
    {
        while (next < total && next - base < XFER_WINDOW)
        {
            FSIZE_t pos = (FSIZE_t)next * XFER_CHUNK_SIZE;
            UINT count = 0;

            if (f_tell(&x->file) != pos && (res = f_lseek(&x->file, pos)) != FR_OK)
                return res;
            if ((res = f_read(&x->file, TX_PAYLOAD, XFER_CHUNK_SIZE, &count)) != FR_OK)
                return res;

            // Chunks are resent after errors, only count each one once
            if (next == crc_next)
            {
                crc = crc32_le(crc, TX_PAYLOAD, count);
                crc_next++;
            }

            send_frame(CMD_DATA, next, NULL, count);
            next++;
        }

        draw_progress(path, (uint64_t)base * XFER_CHUNK_SIZE, size, started, false);

        if (exit_requested())
            return ST_ABORTED;

        int ret = recv_frame(&x->req, 1000);
        if (ret == 1 && (x->req.cmd == CMD_ACK || x->req.cmd == CMD_NAK))
        {
            uint32_t index = base + (uint16_t)(x->req.seq - (uint16_t)base);

            if (x->req.len >= 1 && x->req.data[0] != ST_OK)
                return ST_ABORTED;

            if (x->req.cmd == CMD_ACK && index < next)
            {
                base = index + 1;
                timeouts = 0;
            }
            else if (x->req.cmd == CMD_NAK && index <= next)
            {
                base = next = index;
            }
        }
        else if (ret == 1)
        {
            // The host started something else, it gave up on this transfer
            x->pending = true;
            return ST_ABORTED;
        }
        else if (ret == 0)
        {
            if (++timeouts > 10)
                return ST_TIMEOUT;
            next = base;
        }
    }

    draw_progress(path, size, size, started, true);
    *crc_out = crc;
    return ST_OK;
}

static void cmd_get(const frame_t *req)
{
    char *path = x->path[0];
    uint16_t seq = req->seq;
    uint32_t crc = 0;
    FRESULT res;

    if (!make_path(path, req->data, req->len))
    {
        send_status(req, ST_BAD_PATH);
        return;
    }

    if ((res = f_open(&x->file, path, FA_READ)) != FR_OK)
    {
        send_status(req, res);
        return;
    }

    uint64_t size = f_size(&x->file);
    put_u64(TX_PAYLOAD + 1, size);
    send_reply(req, req->cmd, ST_OK, 8);

    snprintf(x->header, sizeof(x->header), "Sending to PC");
    uint8_t status = send_file(path, size, &crc);
    f_close(&x->file);

    // Reply: status, crc32 of the whole file
    if (!x->pending)
    {
        TX_PAYLOAD[0] = status;
        put_u32(TX_PAYLOAD + 1, crc);
        send_frame(CMD_GET_END, seq, NULL, 5);
    }

    snprintf(x->header, sizeof(x->header), "Connected (%u bps)", x->baud);
    draw_status(status == ST_OK ? "Transfer complete" : "Transfer failed", -1, true);
}

// Receives CMD_DATA frames into the open file until CMD_PUT_END. Returns a status code.
static uint8_t recv_file(const char *path, uint64_t size)
{
    uint32_t expected = 0;
    uint32_t crc = 0;
    uint64_t written = 0;
    bool nak_sent = false;
    TickType_t started = xTaskGetTickCount();
    TickType_t last_frame = started;
    FRESULT res;

    while (true)
    {
        draw_progress(path, written, size, started, false);

        if (exit_requested())
            return ST_ABORTED;

        int ret = recv_frame(&x->req, 200);
        if (ret == 0)
        {
            if (time_reached(last_frame + pdMS_TO_TICKS(TRANSFER_IDLE_MS)))
                return ST_TIMEOUT;
            continue;
        }

        if (ret < 0)
        {
            if (!nak_sent)
                send_ack(CMD_NAK, expected, ST_OK);
            nak_sent = true;
            continue;
        }

        last_frame = xTaskGetTickCount();

        if (x->req.cmd == CMD_DATA)
        {
            uint16_t seq = x->req.seq;

            if (seq == (uint16_t)expected)
            {
                UINT count = 0;

                if (written + x->req.len > size)
                {
                    send_ack(CMD_ACK, seq, ST_SIZE_MISMATCH);
                    return ST_SIZE_MISMATCH;
                }

                if ((res = f_write(&x->file, x->req.data, x->req.len, &count)) != FR_OK || count != x->req.len)
                {
                    res = res ? res : FR_DENIED; // Disk full
                    send_ack(CMD_ACK, seq, res);
                    return res;
                }

                crc = crc32_le(crc, x->req.data, x->req.len);
                written += x->req.len;
                send_ack(CMD_ACK, seq, ST_OK);
                expected++;
                nak_sent = false;
            }
            else if ((uint16_t)(expected - seq) <= XFER_WINDOW * 2)
            {
                // Duplicate of a chunk already written, our ACK got lost
                send_ack(CMD_ACK, expected - 1, ST_OK);
            }
            else if (!nak_sent)
            {
                send_ack(CMD_NAK, expected, ST_OK);
                nak_sent = true;
            }
        }
        else if (x->req.cmd == CMD_PUT_END && x->req.len >= 4)
        {
            draw_progress(path, written, size, started, true);
            if (written != size)
                return ST_SIZE_MISMATCH;
            if (crc != get_u32(x->req.data))
                return ST_CRC_MISMATCH;
            return ST_OK;
        }
        else
        {
            // The host started something else, it gave up on this transfer
            x->pending = true;
            return ST_ABORTED;
        }
    }
}

static void cmd_put(const frame_t *req)
{
    // Payload: size u64, path. We write to <path>.part and rename once the whole file is verified.
    char *path = x->path[0];
    char *temp = x->path[1];
    FRESULT res;

    if (req->len < 9 || !make_path(path, req->data + 8, req->len - 8) || is_root(path))
    {
        send_status(req, ST_BAD_PATH);
        return;
    }

    uint64_t size = get_u64(req->data);
    size_t path_len = strlen(path);
    memcpy(temp, path, path_len);
    memcpy(temp + path_len, ".part", 6);

    if ((res = f_open(&x->file, temp, FA_WRITE | FA_CREATE_ALWAYS)) != FR_OK)
    {
        send_status(req, res);
        return;
    }

    send_status(req, ST_OK);

    snprintf(x->header, sizeof(x->header), "Receiving from PC");
    uint8_t status = recv_file(path, size);

    if ((res = f_close(&x->file)) != FR_OK && status == ST_OK)
        status = res;

    if (status == ST_OK)
    {
        res = f_unlink(path);
        if (res != FR_OK && res != FR_NO_FILE)
            status = res;
        else
            status = f_rename(temp, path);
    }

    if (status != ST_OK)
        f_unlink(temp);

    // recv_file left the PUT_END frame in x->req, unless the host gave up.
    // If we gave up (B pressed, timeout), a failed ACK tells the host to stop sending.
    if (!x->pending && x->req.cmd == CMD_PUT_END)
        send_status(&x->req, status);
    else if (!x->pending && status != ST_OK)
        send_ack(CMD_ACK, 0, status);

    snprintf(x->header, sizeof(x->header), "Connected (%u bps)", x->baud);
    draw_status(status == ST_OK ? "Transfer complete" : "Transfer failed", -1, true);
}

// Returns true if the session should end to boot the installed app
static bool cmd_install(const frame_t *req)
{
    // Payload: boot u8, path
    char *path = x->path[0];
    const char *error;

    if (req->len < 2 || !make_path(path, req->data + 1, req->len - 1) || is_root(path))
    {
        send_status(req, ST_BAD_PATH);
        return false;
    }

    // The installer wants a VFS path
    snprintf(path, sizeof(x->path[0]), SDCARD_BASE_PATH "%.*s", req->len - 1, req->data + 1);

    if ((error = x->hooks->install(path)))
    {
        send_error(req, ST_INSTALL_FAILED, error);
        draw_status("Install failed", -1, true);
        return false;
    }

    send_status(req, ST_OK);
    draw_status("Install complete", -1, true);
    return req->data[0] != 0;
}


static void cmd_apps(const frame_t *req)
{
    // Reply: status, count u8, entries [start u32, end u32, seq u16, desc_len u8, desc, file_len u8, file]
    usbxfer_app_t *apps = calloc(XFER_MAX_APPS, sizeof(usbxfer_app_t));
    uint8_t *p = TX_PAYLOAD + 2;

    if (!apps)
    {
        send_status(req, FR_NOT_ENOUGH_CORE);
        return;
    }

    int count = x->hooks->list_apps(apps, XFER_MAX_APPS);

    for (int i = 0; i < count; i++)
    {
        size_t desc_len = strlen(apps[i].description);
        size_t file_len = strlen(apps[i].filename);

        put_u32(p, apps[i].start), p += 4;
        put_u32(p, apps[i].end), p += 4;
        put_u16(p, apps[i].seq), p += 2;
        *p++ = desc_len;
        memcpy(p, apps[i].description, desc_len), p += desc_len;
        *p++ = file_len;
        memcpy(p, apps[i].filename, file_len), p += file_len;
    }

    free(apps);

    TX_PAYLOAD[1] = count;
    send_reply(req, req->cmd, ST_OK, p - (TX_PAYLOAD + 1));
}

static void cmd_erase(const frame_t *req)
{
    // Payload: start u32, flags u8 (bit 0: only erase the app's NVS)
    const char *error;

    if (req->len < 5)
    {
        send_status(req, ST_BAD_REQUEST);
        return;
    }

    if ((error = x->hooks->erase_app(get_u32(req->data), req->data[4] & 1)))
    {
        send_error(req, ST_APP_ERROR, error);
        draw_status("Erase failed", -1, true);
        return;
    }

    send_status(req, ST_OK);
    draw_status(req->data[4] & 1 ? "App data erased" : "App removed", -1, true);
}


usbxfer_result_t usbxfer_run(const usbxfer_hooks_t *hooks, bool host_started)
{
    usbxfer_result_t result = USBXFER_EXIT;
    bool running = true;

    usbxfer_init();

    if (!driver_installed || !(x = calloc(1, sizeof(xfer_t))))
        return USBXFER_EXIT;

    x->hooks = hooks;
    x->baud = XFER_DEFAULT_BAUD;
    x->buttons = input_get_state();

    // The console shares the UART, any log output would corrupt the frames
    esp_log_level_set("*", ESP_LOG_NONE);

    snprintf(x->header, sizeof(x->header), host_started ? "Connecting..." : "Waiting for PC...");
    draw_status("Run tools/sdxfer.py on the PC", -1, true);

    TickType_t last_frame = xTaskGetTickCount();

    while (running)
    {
        int ret = x->pending ? 1 : recv_frame(&x->req, 100);
        x->pending = false;

        if (exit_requested())
        {
            // Let the host know we are gone
            send_frame(CMD_BYE, 0, NULL, 0);
            break;
        }

        if (ret != 1)
        {
            if (x->baud != XFER_DEFAULT_BAUD && x->baud_deadline && time_reached(x->baud_deadline))
            {
                // The host never talked to us at the new speed
                uart_set_baudrate(XFER_UART, XFER_DEFAULT_BAUD);
                x->baud = XFER_DEFAULT_BAUD;
                x->baud_deadline = 0;
                snprintf(x->header, sizeof(x->header), "Waiting for PC...");
                draw_status("Run tools/sdxfer.py on the PC", -1, true);
            }
            if (host_started && time_reached(last_frame + pdMS_TO_TICKS(HOST_IDLE_MS)))
                break;
            continue;
        }

        last_frame = xTaskGetTickCount();
        x->baud_deadline = 0;

        switch (x->req.cmd)
        {
            case CMD_HELLO:     cmd_hello(&x->req); break;
            case CMD_SET_BAUD:  cmd_set_baud(&x->req); break;
            case CMD_PING:      cmd_ping(&x->req); break;
            case CMD_STATFS:    cmd_statfs(&x->req); break;
            case CMD_LIST:      cmd_list(&x->req); break;
            case CMD_STAT:      cmd_stat(&x->req); break;
            case CMD_MKDIR:
            case CMD_RMDIR:
            case CMD_RM:        cmd_path_op(&x->req); break;
            case CMD_RENAME:    cmd_rename(&x->req); break;
            case CMD_GET:       cmd_get(&x->req); break;
            case CMD_PUT:       cmd_put(&x->req); break;
            case CMD_INSTALL:
                if (cmd_install(&x->req))
                {
                    result = USBXFER_BOOT_INSTALLED;
                    running = false;
                }
                break;
            case CMD_APPS:      cmd_apps(&x->req); break;
            case CMD_ERASE:     cmd_erase(&x->req); break;
            case CMD_BYE:
                send_status(&x->req, ST_OK);
                running = false;
                break;
            case CMD_DATA:
            case CMD_ACK:
            case CMD_NAK:
            case CMD_PUT_END:
                // Leftovers of an aborted transfer
                break;
            default:
                send_status(&x->req, ST_BAD_REQUEST);
                break;
        }
    }

    uart_wait_tx_done(XFER_UART, pdMS_TO_TICKS(500));
    uart_set_baudrate(XFER_UART, XFER_DEFAULT_BAUD);
    uart_flush_input(XFER_UART);
    magic_state = 0;

    esp_log_level_set("*", CONFIG_LOG_DEFAULT_LEVEL);

    free(x);
    x = NULL;

    return result;
}
