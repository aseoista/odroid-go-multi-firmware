#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t start;     // Flash offset, also identifies the app
    uint32_t end;
    uint16_t seq;       // Install order
    char description[40];
    char filename[40];
} usbxfer_app_t;

typedef struct
{
    // Draw the transfer screen. percent < 0 hides the progress bar.
    void (*status)(const char *header, const char *message, int percent);
    // Install a .fw file (full VFS path). Returns NULL on success or an error message.
    const char *(*install)(const char *path);
    // Fill out with the installed apps, returns how many
    int (*list_apps)(usbxfer_app_t *out, int max);
    // Remove the app starting at this flash offset (or only erase its NVS). Returns NULL or an error message.
    const char *(*erase_app)(uint32_t start, bool nvs_only);
} usbxfer_hooks_t;

typedef enum
{
    USBXFER_EXIT = 0,
    USBXFER_BOOT_INSTALLED, // The host asked to boot the app that was just installed
} usbxfer_result_t;

// Install the UART driver used for transfers. Call once before using the functions below.
void usbxfer_init(void);
// Non-blocking. Returns true if the host is trying to start a session.
bool usbxfer_host_detected(void);
// Blocking transfer session. Returns when B is pressed, the host disconnects, or a boot is requested.
usbxfer_result_t usbxfer_run(const usbxfer_hooks_t *hooks, bool host_started);
