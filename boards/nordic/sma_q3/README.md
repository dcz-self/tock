Platform-Specific Instructions: SMA Q3
===================================

The SMA Q3 is the smart watch used in [Jazda 2.0](https://jazda.org) and Bangle.js 2.
It is a platform based around the nRF52840, an SoC with an ARM Cortex-M4 and a BLE radio.
The smart watch exposes 2 SWD pins, and includes 1 button, 1 backlight LED and a lot of assorted peripherals.

## Getting Started

To program the SMA Q3 with Tock, you will need a STLink 2.0 device and the
appropriate cables. An example setup is going to be available on the Jazda website.

You'll also need to install [OpenOCD](../../../doc/Getting_Started.md) to proress with the programming.

Then, follow the [Tock Getting Started guide](../../../doc/Getting_Started.md)

## Programming the kernel

Once you have all software installed, you should be able to simply run
make flash in this directory to install a fresh kernel.

Alternatively:

```
tockloader flash --address 0 --openocd --board sma_q3 .../tock_build/target/thumbv7em-none-eabi/release/sma_q3.bin
```

## Programming user-level applications
You can program an application via JTAG using `tockloader`:

```shell
$ cd libtock-c/examples/<app>
$ make
$ tockloader install --openocd --board sma_q3
```

## Debugging

See the [nrf52dk README](../nrf52dk/README.md) for information about debugging
the SMA Q3. Note: may need adjustment due to missing serial interface.
