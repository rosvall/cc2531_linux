# Linux kernel driver for CC2531 WPAN Adapter
IEEE 802.15.4 Linux driver for [CC2531 based USB dongle](https://www.ti.com/tool/CC2531USB-RD) with [WPAN Firmware](https://github.com/rosvall/cc2531_usb_wpan_adapter).

## Features
- Send and receive IEEE 802.15.4 frames.
- Get/set channel, address, transmit power, CCA mode, etc.
- Promiscous mode.
- Exposes all hardware registers in sysfs for experimentation.

## Requirements
- CC2531 based USB dongle with [WPAN Firmware](https://github.com/rosvall/cc2531_usb_wpan_adapter)
- Linux 6.0 or newer
- GCC, binutils, kernel headers, etc.

## How to build
```sh
# Clone repo
git clone 'https://github.com/rosvall/cc2531_linux.git' 
cd cc2531_linux

# Build
make

# Install module
sudo make modules_install
```

## References
 - [CC253x/4x User's Guide (Rev. D)](https://www.ti.com/lit/pdf/swru191)

## See also
 - [Flash a stock Texas Instruments CC2531USB-RD dongle, no tools required](https://github.com/rosvall/cc2531_oem_flasher)
 - [Simple USB DFU bootloader for TI CC2531](https://github.com/rosvall/cc2531_bootloader)
 - [WPAN Adapter firmware for CC2531 USB Dongle](https://github.com/rosvall/cc2531_usb_wpan_adapter)
