# Tomu Quickstart Guide

This guide describes everything you need to set up your system to develop for [Tomu](https://tomu.im/).

This quickstart guide is designed to be used on Mac, Windows, Linux, and anything else that can run GCC and Make.

The [main Tomu U2F firmware](https://github.com/im-tomu/chopstx/tree/efm32/u2f) is in a different repo.

## Overview of Requirements

To build and load sample code, you will need three things:

1. An [ARM toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
1. Make
1. dfu-utils

Installation varies depending on your platform:

Platform   | ARM Toolchain  | Make  | dfu-util
---------- | -------------- | ----- | ----------
*Windows*    | [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) | [GNU Win32 Make](http://gnuwin32.sourceforge.net/packages/make.htm) | [precompiled binaries](http://dfu-util.sourceforge.net/releases/dfu-util-0.8-binaries/win32-mingw32/)
*macOS*      | [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) | [Xcode](https://itunes.apple.com/us/app/xcode/id497799835) | [Homebrew](https://brew.sh/) `brew install dfu-util`
*Debian/Ubuntu* | `sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib` | `sudo apt-get install make` | `sudo apt-get install dfu-util`
*Fedora* | `sudo dnf install arm-none-eabi-newlib arm-none-eabi-gcc-cs-c++` | `sudo dnf install make` | `sudo dnf install dfu-util`
*Arch* | `sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib` | `sudo pacman -S make` | `sudo pacman -S dfu-util`

This quickstart repo differs from the samples repo in that it has a prebuilt version of `libopencm3`, which normally requires various command line programs to compile.  This cuts down on compile time, and enables building on platforms that don't have commands like grep, printf, or cat.

## Building Examples

To build an example, go into the directory and type `make`. For example, `bare-minimum`.

## Loading Examples

### Upgrade toboot (if needed)
Before you start, check if you have an old bootloader. On linux, it will look like this in dmesg:
"Product: Tomu Bootloader" instead of "Product: Tomu Bootloader (5) v2.0-rc7"  
If you flash a program, you will lose the bootloader and require a recovery procedure to get the bootloader back.  
Either way, you need to go to: https://github.com/im-tomu/toboot . If you lost your bootloader, you will have to
short external pads 1 and 4 before you insert tomu. After that, you'll need to flash 2 files:
* "sudo dfu-util --download toboot-boosted.dfu" (flash, and then re-insert tomu)
* tomu should now look like this in dmesg "Tomu Bootloader (1) v2.0-rc7"
* flash a 2nd time with "sudo dfu-util -D toboot-boosted.bin"
* now tomu should say "Tomu Bootloader (5) v2.0-rc7"

### Load an an example
To load examples onto Tomu, ensure it is in DFU mode by verifying that the red and green LEDs are alternately blinking, and that it shows up if you run `sudo dfu-util --list`.  Then, load the sample you want using `dfu-util --download project.dfu`.

Note that you may need to use sudo or proper udev permissions to flash:
```
sauron [mc]$ dfu-util --download miniblink.dfu
Match vendor ID from file: 1209
Match product ID from file: 70b1
dfu-util: Cannot open DFU device 1209:70b1
dfu-util: No DFU capable USB device available

sauron [mc]$ sudo dfu-util --download miniblink.dfu
Match vendor ID from file: 1209
Match product ID from file: 70b1
Opening DFU capable USB device...
ID 1209:70b1
Run-time device DFU version 0101
Claiming USB DFU Interface...
Setting Alternate Setting #0 ...
Determining device status: state = dfuIDLE, status = 0
dfuIDLE, continuing
DFU mode device DFU version 0101
Device returned transfer size 1024
Copying data from PC to DFU device
Download	[=========================] 100%         1164 bytes
```

### Permissions and udev
If you need to run sudo every time, on linux you can add some udev rules change permissions automatically.
Write this into /etc/udev/rules.d/10-tomu.rules, then run 'udevadm trigger', and re-insert the device.
```
# Device #1, unknown?
ACTION=="add|change", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="cdab", TAG+="uaccess"
# Device #2: Tomu old and new bootloaders:
ACTION=="add|change", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="70b1", TAG+="uaccess"
```

### Loading the next program, or "help, I can't load the next program"
To load another program, unplug Tomu and plug it back in.  If you do not get the tomu bootloader when you do so,
jump back to the beginning of this section and see how to force boot into toboot and upgrade it.

## Creating a new Project

To create a new project, simply copy an existing project.  The [`bare-minimum`](./bare-minimum) project is a good example if you want to start from scratch.

The new project's .dfu file will be based on the directory name.

## Troubleshooting Tips

* The `miniblink` program, when correctly operating, looks quite similar to the bootloader's "waiting for instructions" state. You can change this by editing the `system_millis` check in the `sys_tick_handler` function to make the LEDs flash faster or slower.

* Early versions of the bootloader only work with programs that have a toboot-v2.0 signature.  If you load a program and get a stream of `test-in-progress` over the USB serial console instead of what you expect the program to do, you should [upgrade toboot](https://github.com/im-tomu/tomu-bootloader#installing-or-upgrading-toboot).  To add a toboot-v2.0 signature, add the following near the top of the program and recompile:
  ```
  // Make this program compatible with Toboot-V2.0
  #include <toboot.h>
  TOBOOT_CONFIGURATION(0);
  ```
