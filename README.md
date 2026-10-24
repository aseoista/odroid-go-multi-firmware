# Description
Odroid-go-multi-firmware is an improvement of the official Odroid GO firmware. It allows you to keep multiple applications installed in the flash and switch instantly between them. You can think of it as a multi-boot loader.


# Usage
Holding **B** while powering up the device will bring you to the boot menu.


# Installation

_Note: There is no risk in flashing your Odroid GO. You can always recover and return to the stock firmware by following ODROID's guide (method 2 below)._

### Method 1: Self-installer

This is the easiest method, no need for drivers or flashing software. But it isn't as well tested.

1. Copy the file `odroid-go-multi-firmware-XXX.fw` to your sdcard in the folder `/odroid/firmware`
2. Power up your device while holding B
3. Flash the self-installer and run it


### Method 2: USB flashing

Follow these instructions: https://wiki.odroid.com/odroid_go/firmware_update but use the .img provided here.

_Note: If you are familiar with esptool, you just have to flash `odroid-go-multi-firmware-XXX.fw` to offset 0x0._


# USB file transfer
While the boot menu is shown, you can access the SD card from a PC over the USB cable with `tools/sdxfer.py` (requires `pip install pyserial`). The device switches to transfer mode by itself when the tool connects and goes back to the menu when it is done. You can also open it from MENU > USB File Transfer, and press B to leave it.

```
python tools/sdxfer.py info                                   # device, firmware and SD card info
python tools/sdxfer.py ls -l /roms/nes                        # list a directory
python tools/sdxfer.py put game.nes /roms/nes                 # upload (put -r for directories)
python tools/sdxfer.py get -r /retro-go/saves ./backup        # download
python tools/sdxfer.py put -r --skip ./roms /                 # only upload files missing on the card
python tools/sdxfer.py rm /roms/nes/game.nes                  # also: mkdir [-p], rmdir, mv, df
python tools/sdxfer.py install --boot app.fw                  # upload, install and boot a .fw, no button press
python tools/sdxfer.py apps                                   # list installed apps
python tools/sdxfer.py erase 2                                # remove an app (number, name or .fw file name)
python tools/sdxfer.py erase --nvs "OGO Keyboard"             # only erase an app's saved settings (NVS)
python tools/sdxfer.py menu                                   # reboot into the boot menu (requires esptool)
```

- The port defaults to `/dev/ttyUSB0`, use `-p` or the `SDXFER_PORT` environment variable to change it.
- The fastest reliable speed (up to 2 Mbaud) is negotiated automatically, CH34x USB chips for example end up at 1.5 Mbaud. Expect 100-150 KB/s, it's best for ROMs, saves and firmware rather than whole collections.
- Uploads are written to a `.part` file and renamed once verified, an interrupted transfer never leaves a damaged file.
- Paths are relative to the card root and must be plain ASCII. Files with other names are skipped with a warning.
- `menu` erases otadata, like holding B at power-up. It needs a board with the usual DTR/RTS auto-reset circuit.


# Compilation
The official esp-idf version 3.1 or newer is required and to improve SD Card support you should this patch:

- tools/patches/esp-idf-sdcard-patch.diff

_Note: Those patches do not introduce breaking changes to non-GO (standard ESP32) projects and can safely be applied to your global esp-idf installation._

## Build Steps:
1. Compile firmware: `idf.py build`
2. And then:
   - To produce image files: `python tools/pack.py`
   - To flash and debug: `idf.py flash monitor`


# Technical information

### Creating .fw files
The mkfw.py tool is used to package your application in a .fw file.

Usage:    
`mkfw.py output_file.fw 'description' tile.raw type subtype size label file.bin [type subtype size label file.bin, ...]`

- tile.raw must be a RAW RGB565 86x48 image

- [type](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html#type) is 0 for application and 1 for data
- [subtype](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html#subtype) can be set to 0 for auto
- [size](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html#offset-size) must be equal to or bigger than your app's .bin and a multiple of 65536. Can be set to 0 for auto
- [label](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/partition-tables.html#name-field) is a label for your own usage
- file.bin contains the partition's data

### .fw format:
```
 Header:
   "ODROIDGO_FIRMWARE_V00_01"  24 bytes
   Firmware Description        40 bytes
   RAW565 86x48 tile           8256 bytes
 Partition [, ...]:
   Type                        1 byte
   Subtype                     1 byte
   Padding                     2 bytes
   Label                       16 bytes
   Flags                       4 bytes
   Size                        4 bytes
   Data length                 4 bytes
   Data                        <Data length> bytes
 Footer:
   CRC32                       4 bytes
```

# Questions

> **Q: How does it work?**
>
> A: When you select an application to boot, the ESP32's partition table is rewritten to make it seem like the app is the only thing there. That way any existing, unmodified, Odroid-GO applications will work. The firmware keeps track of what is installed and where. The app will then boot every time you power up the system, until you change it.
