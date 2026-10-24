#!/usr/bin/env python3
"""Transfer files between the PC and the SD card of a device running odroid-go-multi-firmware.

The device must be in the multi-firmware menu (hold B while powering it up, or use the `menu`
command). It enters transfer mode automatically when this tool connects. See main/usbxfer.c
for the protocol.
"""
import argparse, calendar, os, posixpath, signal, struct, subprocess, sys, time, zlib

try:
    import serial
except ImportError:
    exit("This tool requires pyserial: pip install pyserial")

PROTO_VERSION = 1
DEFAULT_BAUD = 115200
BAUD_RATES = [2000000, 1500000, 921600, 460800, 230400]
LINK_TEST_FRAMES = 8  # Full-size frames the device must echo cleanly before we accept a baud rate
BAUD_PROBATION = 2.0  # The device reverts to DEFAULT_BAUD after this many seconds of silence
MAGIC = b"\xA5\x5A"
MAX_PAYLOAD = 4096 + 16

CMD_HELLO, CMD_SET_BAUD, CMD_PING, CMD_BYE = 0x01, 0x02, 0x03, 0x04
CMD_STATFS, CMD_LIST, CMD_STAT, CMD_MKDIR, CMD_RMDIR, CMD_RM, CMD_RENAME = range(0x10, 0x17)
CMD_GET, CMD_GET_END, CMD_PUT, CMD_PUT_END = range(0x20, 0x24)
CMD_INSTALL, CMD_APPS, CMD_ERASE = 0x28, 0x29, 0x2A
CMD_DATA, CMD_ACK, CMD_NAK = 0x30, 0x31, 0x32
CMD_REPLY = 0x80

STATUS_NAMES = {
    1: "disk error", 2: "internal error", 3: "card not ready", 4: "no such file",
    5: "no such directory", 6: "invalid name", 7: "access denied (directory not empty or disk full?)",
    8: "already exists", 9: "invalid object", 10: "write protected", 11: "invalid drive",
    12: "card not mounted", 13: "no filesystem", 14: "mkfs aborted", 15: "timeout",
    16: "locked", 17: "not enough memory", 18: "too many open files", 19: "invalid parameter",
    100: "bad request", 101: "bad path (must be absolute and plain ASCII)", 102: "CRC mismatch",
    103: "size mismatch", 104: "aborted on the device", 105: "install failed", 106: "timeout",
    107: "not a file", 108: "not a directory", 109: "app operation failed",
}
FR_NO_FILE, FR_NO_PATH, FR_EXIST = 4, 5, 8

BAD_FRAME = object()


class UsageError(Exception):
    pass


class XferError(Exception):
    def __init__(self, status, detail=b""):
        self.status = status
        message = STATUS_NAMES.get(status, "error %d" % status)
        if detail:
            message += ": " + detail.decode(errors="replace")
        super().__init__(message)


class Frame:
    def __init__(self, cmd, seq, payload):
        self.cmd, self.seq, self.payload = cmd, seq, payload


class Device:
    def __init__(self, port, timeout=5.0):
        self.port = port
        self.timeout = timeout
        self.buf = bytearray()
        self.seq = 0
        self.connected = False
        self.sending = False
        self.interrupted = False
        self.ser = serial.Serial(port, DEFAULT_BAUD, timeout=0.1)
        # The boards have the esptool auto-reset circuit: EN is pulled low while RTS is asserted
        # without DTR. Release RTS first so we never end up in that state.
        self.ser.rts = False
        self.ser.dtr = False
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def on_sigint(self, signum, frame):
        # A frame cut in half would swallow the next ones (like our BYE): finish sending it first
        if self.sending:
            self.interrupted = True
        else:
            raise KeyboardInterrupt

    # Framing

    def send(self, cmd, seq, payload=b""):
        header = struct.pack("<BHH", cmd, seq & 0xFFFF, len(payload))
        crc = zlib.crc32(header + payload) & 0xFFFFFFFF
        self.sending = True
        try:
            self.ser.write(MAGIC + header + payload + struct.pack("<I", crc))
        finally:
            self.sending = False
        if self.interrupted:
            self.interrupted = False
            raise KeyboardInterrupt

    def _parse(self):
        start = self.buf.find(MAGIC)
        if start < 0:
            del self.buf[:-1]  # Keep a possible first half of the magic
            return None
        del self.buf[:start]
        if len(self.buf) < 7:
            return None
        cmd, seq, length = struct.unpack_from("<BHH", self.buf, 2)
        if length > MAX_PAYLOAD:
            del self.buf[:2]
            return BAD_FRAME
        if len(self.buf) < 7 + length + 4:
            return None
        frame = bytes(self.buf[2 : 7 + length])
        (crc,) = struct.unpack_from("<I", self.buf, 7 + length)
        if zlib.crc32(frame) & 0xFFFFFFFF != crc:
            del self.buf[:2]
            return BAD_FRAME
        del self.buf[: 7 + length + 4]
        return Frame(cmd, seq, frame[5:])

    def recv(self, timeout):
        """Returns a Frame, BAD_FRAME, or None on timeout. Raises if the device leaves the session."""
        deadline = time.monotonic() + timeout
        while True:
            frame = self._parse()
            if frame is not None:
                if frame is not BAD_FRAME and frame.cmd == CMD_BYE:
                    self.connected = False
                    raise ConnectionError("The session was closed on the device")
                return frame
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                # An incomplete frame never got its tail: skip its magic so we can resync
                if self.buf.startswith(MAGIC):
                    del self.buf[:2]
                return None
            self.ser.timeout = min(remaining, 0.1)
            self.buf += self.ser.read(max(1, self.ser.in_waiting))

    def request(self, cmd, payload=b"", timeout=None, retries=3):
        """Sends a request and returns the reply's payload (after the status byte)."""
        self.seq = (self.seq + 1) & 0xFFFF
        timeout = timeout or self.timeout
        for _ in range(retries):
            self.send(cmd, self.seq, payload)
            deadline = time.monotonic() + timeout
            while True:
                frame = self.recv(max(0, deadline - time.monotonic()))
                if frame is None:
                    break
                if frame is BAD_FRAME or frame.cmd != (cmd | CMD_REPLY) or frame.seq != self.seq:
                    continue
                if not frame.payload or frame.payload[0] != 0:
                    raise XferError(frame.payload[0] if frame.payload else 100, frame.payload[1:])
                return frame.payload[1:]
        raise TimeoutError("No reply from the device")

    # Session

    def connect(self, max_baud):
        local_time = calendar.timegm(time.localtime())
        hello = struct.pack("<BI", PROTO_VERSION, local_time)
        try:
            reply = self.request(CMD_HELLO, hello, timeout=0.5, retries=3)
        except TimeoutError:
            # A previous session may still be running at a higher speed (the tool was killed):
            # end it, the device then goes back to the menu and listens at the default speed.
            self._end_stale_session()
            try:
                reply = self.request(CMD_HELLO, hello, timeout=0.5, retries=max(1, int(self.timeout / 0.5)))
            except TimeoutError:
                raise TimeoutError(
                    "No answer from the device on %s. Is it showing the multi-firmware menu?" % self.port
                )
        proto, dev_max_baud, self.chunk_size, self.window = struct.unpack_from("<BIHB", reply)
        self.target, self.version, self.firmware_dir = reply[8:].decode().split("\0")[:3]
        if proto != PROTO_VERSION:
            raise ConnectionError("Protocol version mismatch (device %d, tool %d)" % (proto, PROTO_VERSION))

        for baud in [b for b in BAUD_RATES if b <= min(max_baud, dev_max_baud)]:
            if self._try_baud(baud):
                break
        self.connected = True
        return self

    def _try_baud(self, baud):
        try:
            self.request(CMD_SET_BAUD, struct.pack("<I", baud), retries=1)
        except (XferError, TimeoutError):
            return False
        switched = time.monotonic()
        self.ser.baudrate = baud
        self.buf.clear()
        try:
            self.request(CMD_PING, timeout=0.3, retries=3)
        except TimeoutError:
            # The device goes back to the default speed when it doesn't hear from us
            self._fall_back(switched + BAUD_PROBATION + 0.3)
            return False
        # Some USB-serial chips (CH34x at 2 Mbaud) drop bytes on long bursts from the device,
        # small pings alone don't show it.
        try:
            for _ in range(LINK_TEST_FRAMES):
                echo = self.request(CMD_PING, struct.pack("<H", self.chunk_size), timeout=0.3, retries=1)
                if len(echo) != self.chunk_size:
                    raise TimeoutError()
            return True
        except TimeoutError:
            pass
        # The device hears us fine, so it won't time out: tell it to go back to the default speed
        self.seq = (self.seq + 1) & 0xFFFF
        self.send(CMD_SET_BAUD, self.seq, struct.pack("<I", DEFAULT_BAUD))
        time.sleep(0.05)
        self._fall_back(time.monotonic() + BAUD_PROBATION + 0.3)
        return False

    def _end_stale_session(self):
        for _ in range(2):  # The first BYE may complete a frame the dead session left half sent
            for baud in BAUD_RATES:
                self.ser.baudrate = baud
                self.send(CMD_BYE, 0)
                self.ser.flush()
            time.sleep(1.1)
        self.ser.baudrate = DEFAULT_BAUD
        self.ser.reset_input_buffer()
        self.buf.clear()

    def _fall_back(self, deadline):
        self.ser.baudrate = DEFAULT_BAUD
        self.ser.reset_input_buffer()
        self.buf.clear()
        try:
            self.request(CMD_PING, timeout=0.3, retries=2)
        except TimeoutError:
            time.sleep(max(0, deadline - time.monotonic()))
            self.ser.reset_input_buffer()
            self.buf.clear()
            self.request(CMD_PING, timeout=0.5)

    def bye(self):
        if not self.connected:
            return
        self.connected = False
        try:
            self.request(CMD_BYE, timeout=1, retries=2)
        except (TimeoutError, ConnectionError, XferError):
            pass

    # Operations

    def statfs(self):
        return struct.unpack("<QQ", self.request(CMD_STATFS))

    def stat(self, path):
        """Returns (is_dir, size, mtime) or None if the path doesn't exist."""
        try:
            is_dir, size, mtime = struct.unpack("<BQI", self.request(CMD_STAT, encode_path(path)))
        except XferError as e:
            if e.status in (FR_NO_FILE, FR_NO_PATH):
                return None
            raise
        return bool(is_dir), size, mtime

    def list(self, path):
        """Returns a list of (name, is_dir, size, mtime)."""
        entries = []
        while True:
            reply = self.request(CMD_LIST, struct.pack("<I", len(entries)) + encode_path(path))
            more, count = struct.unpack_from("<BH", reply)
            pos = 3
            for _ in range(count):
                is_dir, size, mtime, name_len = struct.unpack_from("<BQIB", reply, pos)
                pos += 14
                entries.append((reply[pos : pos + name_len].decode(), bool(is_dir), size, mtime))
                pos += name_len
            if not more:
                return entries

    def mkdir(self, path):
        self.request(CMD_MKDIR, encode_path(path))

    def rmdir(self, path):
        self.request(CMD_RMDIR, encode_path(path))

    def rm(self, path):
        self.request(CMD_RM, encode_path(path))

    def rename(self, src, dst):
        self.request(CMD_RENAME, encode_path(src) + b"\0" + encode_path(dst), retries=1)

    def install(self, path, boot):
        # Erasing and writing the flash can take a while, and the device can't talk meanwhile
        self.request(CMD_INSTALL, bytes([boot]) + encode_path(path), timeout=300, retries=1)
        if boot:
            self.connected = False  # The device is rebooting into the app

    def apps(self):
        """Returns the installed apps as dicts, ordered by flash offset."""
        reply = self.request(CMD_APPS)
        apps, pos = [], 1
        for _ in range(reply[0]):
            start, end, seq, desc_len = struct.unpack_from("<IIHB", reply, pos)
            pos += 11
            description = reply[pos : pos + desc_len].decode(errors="replace")
            pos += desc_len
            file_len = reply[pos]
            filename = reply[pos + 1 : pos + 1 + file_len].decode(errors="replace").lstrip("/")
            pos += 1 + file_len
            apps.append({"start": start, "end": end, "seq": seq, "description": description, "filename": filename})
        return sorted(apps, key=lambda a: a["start"])

    def erase(self, start, nvs_only):
        self.request(CMD_ERASE, struct.pack("<IB", start, 1 if nvs_only else 0), timeout=30, retries=1)

    def put(self, local, remote, progress):
        size = os.path.getsize(local)
        total = (size + self.chunk_size - 1) // self.chunk_size
        self.request(CMD_PUT, struct.pack("<Q", size) + encode_path(remote))

        base = next_chunk = crc_next = timeouts = 0
        crc = 0
        cache = {}
        with open(local, "rb") as f:
            while base < total:
                while next_chunk < total and next_chunk - base < self.window:
                    if next_chunk not in cache:
                        f.seek(next_chunk * self.chunk_size)
                        cache[next_chunk] = f.read(self.chunk_size)
                    data = cache[next_chunk]
                    # Chunks are resent after errors, only count each one once
                    if next_chunk == crc_next:
                        crc = zlib.crc32(data, crc)
                        crc_next += 1
                    self.send(CMD_DATA, next_chunk, data)
                    next_chunk += 1

                frame = self.recv(1.0)
                if frame is None:
                    timeouts += 1
                    if timeouts > 10:
                        raise TimeoutError("The device stopped responding")
                    next_chunk = base
                    continue
                if frame is BAD_FRAME or frame.cmd not in (CMD_ACK, CMD_NAK):
                    continue
                if frame.payload and frame.payload[0] != 0:
                    raise XferError(frame.payload[0])
                index = base + ((frame.seq - base) & 0xFFFF)
                if frame.cmd == CMD_ACK and index < next_chunk:
                    for i in range(base, index + 1):
                        cache.pop(i, None)
                    base = index + 1
                    timeouts = 0
                elif frame.cmd == CMD_NAK and index <= next_chunk:
                    base = next_chunk = index
                progress(min(base * self.chunk_size, size), size)

        self.request(CMD_PUT_END, struct.pack("<I", crc & 0xFFFFFFFF), timeout=10, retries=1)
        progress(size, size)

    def get(self, remote, local, progress):
        (size,) = struct.unpack("<Q", self.request(CMD_GET, encode_path(remote)))
        get_seq = self.seq
        expected = received = stalls = 0
        crc = 0
        nak_sent = False
        # If a frame gets lost we ask for it again quickly, instead of waiting for the device to time out
        frame_time = (self.chunk_size + 11) * 10 / self.ser.baudrate
        stall_timeout = self.window * frame_time + 0.2
        progress(0, size)
        temp = local + ".part"
        try:
            with open(temp, "wb") as f:
                while True:
                    frame = self.recv(stall_timeout)
                    if frame is None:
                        stalls += 1
                        if stalls * stall_timeout > self.timeout * 2:
                            raise TimeoutError("The device stopped sending")
                        self.send(CMD_NAK, expected, b"\0")
                        nak_sent = True
                        continue
                    if frame is BAD_FRAME:
                        if not nak_sent:
                            self.send(CMD_NAK, expected, b"\0")
                            nak_sent = True
                        continue
                    if frame.cmd == CMD_DATA:
                        if frame.seq == expected & 0xFFFF:
                            stalls = 0
                            f.write(frame.payload)
                            crc = zlib.crc32(frame.payload, crc)
                            received += len(frame.payload)
                            self.send(CMD_ACK, expected, b"\0")
                            expected += 1
                            nak_sent = False
                            progress(received, size)
                        elif (expected - frame.seq) & 0xFFFF <= self.window * 2:
                            self.send(CMD_ACK, expected - 1, b"\0")  # Our ACK got lost
                        elif not nak_sent:
                            self.send(CMD_NAK, expected, b"\0")
                            nak_sent = True
                    elif frame.cmd == CMD_GET_END and frame.seq == get_seq:
                        status, dev_crc = struct.unpack("<BI", frame.payload)
                        if status:
                            raise XferError(status)
                        if received != size:
                            raise XferError(103)
                        if dev_crc != crc & 0xFFFFFFFF:
                            raise XferError(102)
                        break
            os.replace(temp, local)
        except BaseException:
            if os.path.exists(temp):
                os.remove(temp)
            raise


def encode_path(path):
    return to_remote(path).encode("ascii")


def to_remote(path):
    return posixpath.normpath("/" + path.replace("\\", "/").lstrip("/"))


def is_ascii(name):
    return all(0x20 <= ord(c) < 0x7F for c in name)


def human_size(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return ("%d %s" if unit == "B" else "%.1f %s") % (n, unit)
        n /= 1024


class Progress:
    def __init__(self, label, quiet):
        self.label, self.quiet = label, quiet
        self.started = time.monotonic()
        self.last_draw = 0
        self.finished = False

    def __call__(self, done, total):
        now = time.monotonic()
        if self.quiet or self.finished or (now - self.last_draw < 0.2 and done < total):
            return
        self.finished = done >= total
        self.last_draw = now
        speed = done / max(now - self.started, 1e-6)
        percent = 100 * done // total if total else 100
        sys.stderr.write("\r%-40.40s %3d%%  %9s  %7s/s " % (self.label, percent, human_size(done), human_size(speed)))
        if done >= total:
            sys.stderr.write("\n")
        sys.stderr.flush()


# Commands


def cmd_info(dev, args):
    total, free = dev.statfs()
    print("Target:   %s" % dev.target)
    print("Firmware: %s" % dev.version)
    print("Baud:     %d" % dev.ser.baudrate)
    print("SD card:  %s free of %s" % (human_size(free), human_size(total)))


def cmd_df(dev, args):
    total, free = dev.statfs()
    print("%-12s %12s %12s %12s" % ("", "Size", "Used", "Free"))
    print("%-12s %12s %12s %12s" % ("SD card", human_size(total), human_size(total - free), human_size(free)))


def cmd_ls(dev, args):
    for path in args.paths or ["/"]:
        st = dev.stat(path)
        if st is None:
            raise XferError(FR_NO_FILE, path.encode())
        entries = dev.list(path) if st[0] else [(posixpath.basename(to_remote(path)), *st)]
        if len(args.paths) > 1:
            print("%s:" % to_remote(path))
        for name, is_dir, size, mtime in sorted(entries, key=lambda e: (not e[1], e[0].lower())):
            if args.long:
                date = time.strftime("%Y-%m-%d %H:%M", time.gmtime(mtime)) if mtime else "-"
                print("%s %10s  %s  %s%s" % ("d" if is_dir else "-", "" if is_dir else size, date, name, "/" if is_dir else ""))
            else:
                print(name + ("/" if is_dir else ""))


def put_one(dev, local, remote, args):
    dev.put(local, remote, Progress(remote, args.quiet))


def cmd_put(dev, args):
    remote = to_remote(args.remote or "/")
    target = dev.stat(remote)
    for local in args.local:
        local = local.rstrip("/\\") or local
        name = os.path.basename(local)
        dest = posixpath.join(remote, name) if (target and target[0]) or len(args.local) > 1 else remote
        if os.path.isdir(local):
            if not args.recursive:
                print("Skipping directory %s (use -r)" % local, file=sys.stderr)
                continue
            put_tree(dev, local, dest, args)
        elif args.skip and dev.stat(dest):
            print("Skipping %s (exists)" % dest, file=sys.stderr)
        else:
            put_one(dev, local, dest, args)


def put_tree(dev, local_root, remote_root, args):
    for dirpath, dirnames, filenames in os.walk(local_root):
        rel = os.path.relpath(dirpath, local_root)
        remote_dir = remote_root if rel == "." else posixpath.join(remote_root, *rel.split(os.sep))
        if not is_ascii(remote_dir):
            print("Skipping %s (non-ASCII name)" % dirpath, file=sys.stderr)
            dirnames[:] = []
            continue
        st = dev.stat(remote_dir)
        if st is None:
            dev.mkdir(remote_dir)
            existing = set()
        elif not st[0]:
            raise XferError(108, remote_dir.encode())
        else:
            existing = {e[0].lower() for e in dev.list(remote_dir)} if args.skip else set()
        for name in sorted(filenames):
            if not is_ascii(name):
                print("Skipping %s (non-ASCII name)" % os.path.join(dirpath, name), file=sys.stderr)
            elif name.lower() in existing:
                print("Skipping %s (exists)" % posixpath.join(remote_dir, name), file=sys.stderr)
            else:
                put_one(dev, os.path.join(dirpath, name), posixpath.join(remote_dir, name), args)


def cmd_get(dev, args):
    local = args.local or "."
    for remote in args.remote:
        remote = to_remote(remote)
        st = dev.stat(remote)
        if st is None:
            raise XferError(FR_NO_FILE, remote.encode())
        name = posixpath.basename(remote) or "sdcard"
        dest = os.path.join(local, name) if os.path.isdir(local) else local
        if st[0]:
            if not args.recursive:
                print("Skipping directory %s (use -r)" % remote, file=sys.stderr)
                continue
            get_tree(dev, remote, dest, args)
        elif args.skip and os.path.exists(dest):
            print("Skipping %s (exists)" % dest, file=sys.stderr)
        else:
            dev.get(remote, dest, Progress(remote, args.quiet))


def get_tree(dev, remote_dir, local_dir, args):
    os.makedirs(local_dir, exist_ok=True)
    for name, is_dir, size, mtime in sorted(dev.list(remote_dir)):
        remote = posixpath.join(remote_dir, name)
        local = os.path.join(local_dir, name)
        if is_dir:
            get_tree(dev, remote, local, args)
        elif args.skip and os.path.exists(local):
            print("Skipping %s (exists)" % local, file=sys.stderr)
        else:
            dev.get(remote, local, Progress(remote, args.quiet))


def cmd_rm(dev, args):
    for path in args.paths:
        st = dev.stat(path)
        if st is None:
            raise XferError(FR_NO_FILE, path.encode())
        if st[0]:
            if not args.recursive:
                raise XferError(107, ("%s is a directory (use -r)" % path).encode())
            rm_tree(dev, to_remote(path))
        else:
            dev.rm(path)


def rm_tree(dev, path):
    for name, is_dir, _, _ in dev.list(path):
        child = posixpath.join(path, name)
        rm_tree(dev, child) if is_dir else dev.rm(child)
    dev.rmdir(path)


def cmd_mkdir(dev, args):
    for path in args.paths:
        if args.parents:
            current = ""
            for part in to_remote(path).strip("/").split("/"):
                current += "/" + part
                st = dev.stat(current)
                if st is None:
                    dev.mkdir(current)
                elif not st[0]:
                    raise XferError(FR_EXIST, current.encode())
        else:
            dev.mkdir(path)


def cmd_rmdir(dev, args):
    for path in args.paths:
        dev.rmdir(path)


def cmd_mv(dev, args):
    dst = to_remote(args.dst)
    st = dev.stat(dst)
    if st and st[0]:
        dst = posixpath.join(dst, posixpath.basename(to_remote(args.src)))
    dev.rename(args.src, dst)


def cmd_install(dev, args):
    remote = posixpath.join(dev.firmware_dir, os.path.basename(args.file))
    put_one(dev, args.file, remote, args)
    print("Installing %s (this can take a minute)..." % remote, file=sys.stderr)
    dev.install(remote, args.boot)
    print("Installed%s." % (", booting it" if args.boot else ""), file=sys.stderr)


def cmd_apps(dev, args):
    apps = dev.apps()
    if not apps:
        print("No apps installed.")
        return
    print("%3s  %-32s %-40s %-21s %s" % ("#", "Name", "File", "Flash", "Size"))
    for i, app in enumerate(apps, 1):
        size = app["end"] + 1 - app["start"]
        flash = "0x%06x-0x%06x" % (app["start"], app["end"])
        print("%3d  %-32s %-40s %-21s %s" % (i, app["description"], app["filename"], flash, human_size(size)))


def same_fw_name(query, stored):
    # The app table keeps only the first 38 characters of the file name
    return query == stored or (len(stored) >= 38 and query.startswith(stored))


def find_app(apps, query):
    """Finds an app by its number in `apps`, its name or its .fw file name (case insensitive)."""
    if query.isdigit():
        if not 1 <= int(query) <= len(apps):
            raise UsageError("There is no app #%s, see `apps`" % query)
        return apps[int(query) - 1]
    q = query.lower()
    matches = [a for a in apps if q == a["description"].lower() or same_fw_name(q, a["filename"].lower())]
    if not matches:
        raise UsageError("No app named '%s', see `apps`" % query)
    if len(matches) > 1:
        numbers = ", ".join("#%d" % (apps.index(a) + 1) for a in matches)
        raise UsageError("Several apps are named '%s' (%s), use its number instead" % (query, numbers))
    return matches[0]


def cmd_erase(dev, args):
    apps = dev.apps()
    targets = [find_app(apps, query) for query in args.apps]  # Resolve all of them before erasing any
    for app in targets:
        dev.erase(app["start"], args.nvs)
        what = "Erased the saved data of" if args.nvs else "Removed"
        print("%s %s (0x%06x)" % (what, app["description"], app["start"]), file=sys.stderr)


def cmd_menu(args):
    """Reboots the device into the multi-firmware menu by erasing otadata, like holding B at power-on."""
    esptool = [sys.executable, "-m", "esptool"]
    if subprocess.run(esptool + ["version"], capture_output=True).returncode != 0:
        exit("This command requires esptool: pip install esptool")
    command = esptool + ["--chip", "esp32", "--port", args.port, "--before", "default_reset",
                         "--after", "hard_reset", "erase_region", "0xd000", "0x2000"]
    if subprocess.run(command).returncode != 0:
        exit("esptool failed. Does the board support automatic reset (DTR/RTS)?")
    print("The device is rebooting into the multi-firmware menu.", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-p", "--port", default=os.environ.get("SDXFER_PORT", "/dev/ttyUSB0"),
                        help="serial port (default: $SDXFER_PORT or /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=BAUD_RATES[0], help="maximum baud rate to negotiate")
    parser.add_argument("-t", "--timeout", type=float, default=5.0, help="connection timeout in seconds")
    parser.add_argument("-q", "--quiet", action="store_true", help="no progress output")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("info", help="show device and SD card information").set_defaults(func=cmd_info)
    sub.add_parser("df", help="show SD card usage").set_defaults(func=cmd_df)

    p = sub.add_parser("ls", help="list directories")
    p.add_argument("-l", "--long", action="store_true", help="show sizes and dates")
    p.add_argument("paths", nargs="*")
    p.set_defaults(func=cmd_ls)

    p = sub.add_parser("put", help="upload files (to a directory if REMOTE exists and is one)")
    p.add_argument("-r", "--recursive", action="store_true", help="upload directories")
    p.add_argument("--skip", action="store_true", help="skip files that already exist on the card")
    p.add_argument("local", nargs="+")
    p.add_argument("remote", help="destination on the card")
    p.set_defaults(func=cmd_put)

    p = sub.add_parser("get", help="download files (to a directory if LOCAL is one)")
    p.add_argument("-r", "--recursive", action="store_true", help="download directories")
    p.add_argument("--skip", action="store_true", help="skip files that already exist locally")
    p.add_argument("remote", nargs="+")
    p.add_argument("local", help="local destination")
    p.set_defaults(func=cmd_get)

    p = sub.add_parser("rm", help="delete files")
    p.add_argument("-r", "--recursive", action="store_true", help="delete directories and their contents")
    p.add_argument("paths", nargs="+")
    p.set_defaults(func=cmd_rm)

    p = sub.add_parser("mkdir", help="create directories")
    p.add_argument("-p", "--parents", action="store_true", help="create parent directories as needed")
    p.add_argument("paths", nargs="+")
    p.set_defaults(func=cmd_mkdir)

    p = sub.add_parser("rmdir", help="delete empty directories")
    p.add_argument("paths", nargs="+")
    p.set_defaults(func=cmd_rmdir)

    p = sub.add_parser("mv", help="move or rename a file or directory")
    p.add_argument("src")
    p.add_argument("dst")
    p.set_defaults(func=cmd_mv)

    p = sub.add_parser("install", help="upload a .fw file and install it (no confirmation on the device)")
    p.add_argument("--boot", action="store_true", help="boot the app once installed")
    p.add_argument("file")
    p.set_defaults(func=cmd_install)

    sub.add_parser("apps", help="list the installed apps").set_defaults(func=cmd_apps)

    p = sub.add_parser("erase", help="remove installed apps (no confirmation on the device)")
    p.add_argument("--nvs", action="store_true", help="only erase the apps' saved data (NVS), keep the apps")
    p.add_argument("apps", nargs="+", metavar="APP", help="number from `apps`, app name or .fw file name")
    p.set_defaults(func=cmd_erase)

    sub.add_parser("menu", help="reboot the device into the multi-firmware menu (requires esptool)").set_defaults(func=None)

    args = parser.parse_args()

    if args.command == "menu":
        return cmd_menu(args)

    try:
        dev = Device(args.port, args.timeout)
    except serial.SerialException as e:
        exit(str(e))

    signal.signal(signal.SIGINT, dev.on_sigint)

    try:
        dev.connect(args.baud)
        args.func(dev, args)
    except (XferError, UsageError, TimeoutError, ConnectionError, OSError) as e:
        exit("\nError: %s" % e)
    except KeyboardInterrupt:
        exit("\nInterrupted")
    finally:
        dev.bye()
        dev.close()


if __name__ == "__main__":
    main()
