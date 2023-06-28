# Linux kernel driver for CC2531 WPAN Adapter firmware

## Requirements
- CC2531 based USB dongle with [WPAN Firmware](https://github.com/rosvall/cc2531_wpan_adapter/)
- Linux 6.0 or newer
- GCC, binutils, kernel headers, etc.

## How to build
```sh
# Check out repo with all sub-modules:
git clone --recursive 'https://github.com/rosvall/cc2531_linux.git' 
cd cc2531_linux

# Build
make

# Install module
sudo make modules_install
```
