# IRBlaster

<p>
  <a href="https://github.com/frinyvonnick/gitmoji-changelog#readme">
    <img alt="Documentation" src="https://img.shields.io/badge/documentation-yes-brightgreen.svg" target="_blank" />
  </a>
  <a href="https://github.com/frinyvonnick/gitmoji-changelog/graphs/commit-activity">
    <img alt="Maintenance" src="https://img.shields.io/badge/Maintained%3F-yes-green.svg" target="_blank" />
  </a>
  <a href="https://github.com/frinyvonnick/gitmoji-changelog/blob/master/LICENSE">
    <img alt="License: MIT" src="https://img.shields.io/badge/License-MIT-yellow.svg" target="_blank" />
  </a>
</p>

IRBlaster is a Linux kernel driver for transmitting IR signals on Raspberry Pi using hardware PWM.

## Features
- Hardware PWM for maximum accuracy and low CPU overhead.
- Implemented as Linux kernel driver with high-precision and time-constrained operations done within the driver.
- Comes with header-only usermode library for simple access as well as an usage example.

## Limitations
- Supports only Raspberry Pi 2 and 3 (and maybe 4?) at the moment (this will change in the near future).
- Currently only GPIO 18 supported.

## Portability
- Should work with any "modern" Raspbian installation running on Raspberry Pi 2/3/(4?)
- Should also work on other distributions (not tested)

## Installation
First make sure you are up-to-date:
```bash
$ apt-get update -y
$ apt-get upgrade -y
```
### Kernel headers

In order to build linux kernel modules, you'll have to have matching kernel headers to the kernel you are running.

If you are on Raspbian, you can just run:

```bash
$ sudo apt-get install raspberrypi-kernel-headers
```
On Debian-based platforms:
```bash
$ sudo apt-get install linux-headers-$(uname -r)
```
On other distributions, you'll have to figure this out yourself.

You can check that the header are successfully installed by running:
```bash
$ ls /lib/modules/$(uname -r)/build
```
If the `build` folder is not found or is empty, headers **have not** been properly installed for your kernel version.

### Non-root access (optional)
The driver will create a character device at `/dev/irblaster`, which by default requires root privileges.

You can make it accessible to non-root users without `sudo` by creating `udev` rule file:
```bash
$ sudo touch /etc/udev/rules.d/99-irblaster.rules
```
and writing inside the file:
```
KERNEL=="irblaster", SUBSYSTEM=="ir", MODE="0666"
```

### Build the driver
Build the driver (and example):
```bash
$ make         # build the driver and user mode example
$ make kernel  # build only the driver
$ make example # build only the user mode example
```

### Load the driver to Kernel
```bash
$ sudo insmod kernel/irblaster.ko
```
You can check from `/var/log/kern.log` whether the driver was successfully loaded.

## Usage
**Make sure your IR led or controlling transistor is plugged into (BCM#) GPIO 18.**

You can send IR transmissions by writing to the device.

NEC-protocol implementation in the `example/test.c`:

```C
#include "../user/ib.h"
 
int main() {
   char *buffer;
   unsigned lpw = 9000;  // leading pulse width
   unsigned lgw = 4500;  // leading gap width
   unsigned opw = 560;   // one pulse width
   unsigned ogw = 1680;  // one gap width 
   unsigned zpw = 560;   // zero pulse width
   unsigned zgw = 560;   // zero gap width
   unsigned utp = 560;   // trailing pulse width
   unsigned f = 38000;   // frequency in hz (NEC: 38 kHz)
   unsigned dc_n = 50;   // Pulse within duty cycle period
   unsigned dc_m = 100;  // Duty cycle period
   char *binary = "11001101001100100000111111110000"; // actual payload to be transmitted

   sendIR(lpw, lgw, opw, ogw, zpw, zgw, utp, f, dc_n, dc_m, binary);
   return 0;
}
```

For detailed example of writing to the device see `user/ib.h` user mode library.

All the variables are 32-bit unsigned integers except the data string which can be up to 200 bytes. 

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Roadmap
### Near future
- Support for different GPIOs
- Support for all Raspberry Pi devices
### Some day
- Direct Memory Access (DMA) support to transmit on any GPIO

## References
- [BCM2835 ARM Peripherals](https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf)
- [Linux Device Drivers, Third Edition](https://lwn.net/Kernel/LDD3/)
## License
[MIT](https://choosealicense.com/licenses/mit/)
