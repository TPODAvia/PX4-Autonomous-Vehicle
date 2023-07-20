It appears that your Raspberry Pi 4 (Pi4) running Ubuntu 20.04 is entering boot mode when you use the USART (Universal Synchronous and Asynchronous serial Receiver and Transmitter) pins, which is a common issue when working with UART (Universal Asynchronous Receiver-Transmitter) on the Raspberry Pi.

Based on the search results, there are two main approaches to solve this problem:

1. **Disabling the Serial Service**

The method involves disabling the serial-getty@ttyS0.service, which uses the mini-UART. This method is necessary because the mini-UART does not perform well under Ubuntu 20.04. You can disable the Serial Service with the following commands:

```bash
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service
```
This information is sourced from [Source 0](https://askubuntu.com/questions/1254376/enable-uart-communication-on-pi4-ubuntu-20-04).

2. **Recompiling the U-boot binary**

This method is more complex and involves recompiling the U-boot binary to prevent U-boot from interrupting autoboot when receiving input on the serial lines. Here's a step-by-step guide:

- Install the necessary dependencies:

```bash
apt-get install git make gcc gcc-aarch64-linux-gnu bison flex
```

- Get the U-boot code base:

```bash
git clone --depth 1 git://git.denx.de/u-boot.git && cd u-boot/
```

- Edit the `rip_4_defconfig` and `rpi.h` files as detailed in [Source 4](https://stackoverflow.com/questions/64416496/gps-serial-console-issue-on-ubuntu-20-04-on-raspberry-pi-4).

- Build the new U-boot binary:

```bash
make rpi_4_defconfig && make
```

- Replace the existing U-boot binary with the newly compiled one:

```bash
mv /boot/firmware/uboot_rpi_4.bin /boot/firmware/uboot_rpi_4.bin.bak
mv u-boot.bin /boot/firmware/uboot_rpi_4.bin
```

Remember to back up your files before making any changes to your system, and consider that recompiling the U-boot binary is a complex task that should only be undertaken if you are comfortable with the process.
