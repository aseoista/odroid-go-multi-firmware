---
name: sdxfer
description: Transfer files to and from the SD card of an ODROID-GO (or MRGC-G32) running odroid-go-multi-firmware over the USB cable, and install, boot, list and remove .fw applications remotely, using tools/sdxfer.py. Use when asked to copy ROMs, saves or other files to/from the device, list or manage the device's SD card, upload and flash/install/boot a .fw app, list installed apps, uninstall/erase an app or its saved data (NVS), or get the device back to the multi-firmware boot menu.
---

# Using tools/sdxfer.py

`tools/sdxfer.py` talks to the multi-firmware boot menu over the device's USB serial port. The device
enters transfer mode by itself when the tool connects and returns to the menu when the command ends,
so no button presses are needed. Each invocation is one session (~0.3–1 s to connect and negotiate speed).

## Prerequisites

- `pip install pyserial` (and `pip install esptool` for the `menu` command).
- The device is connected over USB. Port defaults to `/dev/ttyUSB0`; override with `-p PORT` or
  `SDXFER_PORT`. The user needs access to the port (on Linux, `uucp`/`dialout` group).
- Nothing else may hold the port open (close any `idf.py monitor`, miniterm, etc.). Never run two
  sdxfer commands at the same time on the same port.
- The device must be showing the **multi-firmware boot menu**. If it is running an app (a game,
  an emulator), run `python tools/sdxfer.py menu` first, then wait ~3 s for it to reboot.

## Command syntax

Global options go **before** the subcommand. Use `-q` in scripted/agent use: progress bars are written
to stderr with `\r` and only add noise.

```
python tools/sdxfer.py [-p PORT] [-q] [-t TIMEOUT] [-b MAX_BAUD] <command> ...
```

| Command | What it does |
|---|---|
| `info` | Target (`odroid-go` / `mrgc-g32`), firmware version, negotiated baud, SD free/total |
| `df` | SD card size / used / free |
| `ls [-l] [PATH ...]` | List directories (`-l`: size and date). Default `/` |
| `put [-r] [--skip] LOCAL... REMOTE` | Upload. `REMOTE` is **required** (use `/` for the card root) |
| `get [-r] [--skip] REMOTE... LOCAL` | Download. `LOCAL` is **required** (use `.`) |
| `rm [-r] PATH...` | Delete files (`-r`: directories and their contents) |
| `mkdir [-p] PATH...` | Create directories (`-p`: parents, no error if existing) |
| `rmdir PATH...` | Delete empty directories |
| `mv SRC DST` | Rename/move on the card (into `DST` if it is an existing directory) |
| `install [--boot] FILE.fw` | Upload a .fw to the firmware folder and install it into flash |
| `apps` | List installed apps: number, name, .fw file name, flash range, size |
| `erase [--nvs] APP...` | Remove installed apps (`--nvs`: only erase their saved data) |
| `menu` | Reboot the device into the multi-firmware menu (via esptool, no connection needed) |

### Path rules

- Card paths are absolute from the card root: `/roms/nes/game.nes`. A missing leading `/` is added.
- Names must be **plain printable ASCII**. Non-ASCII files are skipped with a warning during
  recursive transfers and rejected with `bad path` otherwise. Tell the user which files were skipped.
- Destinations behave like `cp`: `put a.nes /roms/nes` puts the file *inside* `/roms/nes` if that
  directory exists, otherwise it creates a file named `/roms/nes`. With several sources, `REMOTE` is
  always treated as a directory. Create directories with `mkdir -p` first when unsure.
- `put -r DIR /dest` / `get -r /dir LOCAL` copy the directory itself into the destination if the
  destination exists (`/dest/DIR`), otherwise the destination becomes the copy.
- `--skip` skips files that already exist at the destination (by name only, no content comparison).
  Without it, existing files are overwritten.

## Common tasks

```bash
# Check the device is reachable and how much space is free
python tools/sdxfer.py -q info

# Upload ROMs (only the missing ones)
python tools/sdxfer.py -q mkdir -p /roms/nes
python tools/sdxfer.py -q put -r --skip ./my-roms/nes /roms

# Back up saves
python tools/sdxfer.py -q get -r /retro-go/saves ./backup

# Install an app and boot it, completely hands-off
python tools/sdxfer.py -q install --boot build/myapp.fw
# ...later, bring the device back to the menu
python tools/sdxfer.py menu && sleep 3
```

Prefer one command with many files over many single-file commands (each command reconnects).

## Installing / flashing applications

`install FILE.fw` uploads the file to the target's firmware folder (`/odroid/firmware/` on ODROID-GO,
`/espgbc/firmware/` on MRGC-G32, overwriting a file with the same name), verifies its CRC and writes it
into a free area of the flash. **There is no confirmation on the device**, so only install files the user
asked for.

- Without `--boot` the app is added to the menu list and the device stays in the menu.
- With `--boot` the device reboots into the app; it then boots that app on every power-up until the user
  holds B at power-on or you run `menu`.
- Installing takes ~10 s to a minute (erase + write); the tool waits up to 300 s. Don't interrupt it.
- Each install adds a new entry to the app list, even for an app that is already installed. To replace
  an app, `install` the new one, then `erase` the old entry (check with `apps` first).
- `odroid-go-multi-firmware-*.fw` (this repo's release file) is the multi-firmware **self-installer**:
  booting it re-flashes the multi-firmware itself. Only do that when the user explicitly wants to update
  the multi-firmware.
- To produce a .fw from an ESP-IDF app see the README (`tools/mkfw.py`).

## Listing and removing applications

```bash
python tools/sdxfer.py -q apps                    # numbered list, ordered by flash offset
python tools/sdxfer.py -q erase 3                 # by number from `apps`
python tools/sdxfer.py -q erase "OGO Keyboard"    # by app name or .fw file name (case insensitive)
python tools/sdxfer.py -q erase --nvs 2           # wipe an app's saved settings, keep the app
```

- Always run `apps` right before `erase` and show the user which entry will go: numbers change
  when apps are added or removed, and duplicates of the same app are common.
- A name matching several apps is rejected; use the number. Nothing is erased if any argument is invalid.
- File names in the app table are cut to 38 characters; the full .fw name still matches.
- `erase` removes the app from the menu and frees its flash for the next install; it can't be undone
  (reinstall from the .fw). `--nvs` deletes the app's settings/saves stored in flash (not files on the SD
  card). There is no confirmation on the device, so only erase what the user asked for.
- Removing the app that was booted last is fine: the device then boots the menu instead of it.

Install failures come back as `Error: install failed: <REASON>` and leave the device usable:
`INVALID FIRMWARE FILE`, `CHECKSUM MISMATCH ERROR` (corrupt file), `NOT ENOUGH FREE SPACE`,
`APP TABLE FULL` (the user must erase apps on the device).

## Speed and timeouts

Expect ~100–150 KB/s (about 1 MB every 7–10 s). The baud rate is negotiated automatically (up to 2 Mbaud;
CH34x USB chips settle at 1.5 Mbaud). Give long-running commands a generous tool timeout (e.g. ~2 min per
10 MB). For multi-GB collections, suggest copying with a card reader instead.

## Results and errors

Exit code 0 means success. On failure the tool prints `Error: <message>` to stderr and exits with 1.

| Error | Meaning / what to do |
|---|---|
| `No answer from the device on PORT...` | Not in the boot menu, or the port is busy. Run `menu`, wait ~3 s, retry. If it persists, ask the user to check the cable and that the device is on. |
| `card not mounted` / `card not ready` | The SD card isn't mounted. It is only mounted at boot: ask the user to reseat the card, then run `menu` (or power-cycle). |
| `no such file` / `no such directory` | Wrong path; check with `ls`. |
| `already exists` | `mkdir`/`mv` target exists (use `mkdir -p`, or pick another name). |
| `access denied (directory not empty or disk full?)` | `rmdir` on a non-empty dir (use `rm -r`), or the card is full (`df`). |
| `aborted on the device` / `The session was closed on the device` | Someone pressed B on the device. Ask before retrying. |
| `not a file ... (use -r)` / `not a directory` | Use `rm -r` for directories, `rm` for files. |
| `No app named ...` / `Several apps are named ...` / `There is no app #N` | Run `apps` and use the number. |
| `app operation failed: APP NOT FOUND` | The list changed since `apps`: list again. |

Interrupted uploads never corrupt the destination: data goes to `<name>.part` and is only renamed over
the original after its CRC is verified. Interrupted downloads delete their partial local file.

To verify a transfer explicitly, download it again and compare hashes, or compare sizes with `ls -l`.

## Safety

- `rm -r`, overwriting `put`s, `install` and `erase` are irreversible on the user's card/flash: only do them when
  asked, and double-check paths (never `rm -r /`).
- Don't flash the device with esptool to "fix" connection problems; `menu` is the only reset you need.
