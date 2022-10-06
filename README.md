# PWJ Strain Logger

## Overview

This is a [Zephyr RTOS](https://www.zephyrproject.org/) project that utilizes a
RTOS to measure the stress/strain exhibited by a Pulse Water Jet (PWJ). It does
this by reading a straingauge that goes to a wheatstone bridge then is filtered
and into a STM32F767 microcrontoller. It utilizes the DMA controller to read
off of the ADC and store to a microSD as fast as possible since the PWJ
operates at 20 kHz or 40 kHz. The accompanying daughter board to the Nucleo
STM32F767ZI is located in the repository
[Nanomagnetics Lab](https://github.com/nanomagneticslab).

## Requirements

You are required to have the PWJStrainLogger daughter board which includes a
microSD card reader.

## Installation

Please see the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/getting_started/index.html)
to set up the build system. For brevity I will include the basic steps.

For ZephyrRTOS v2.7.0 (pwjrtos is built against this), we require cmake >= 3.20.

```
pip install west
west init ~/zephyr
west update
west zephyr-export
cd ~/zephyr/zephyr
git checkout v2.7.0
pip3 install -r ~/pwjrtos/zephyr/scripts/requirements
```

You need to setup an embedded toolchain. `gcc-arm-none-eabi` was used here. It
can be installed via,

```
sudo apt install gcc-arm-none-eabi    # Debian/Ubuntu
sudo dnf install arm-none-eabi-newlib # Fedora/Redhat
sudo port install arm-none-eabi-gcc   # macOS by MacPorts
```

Then you need to add the following *exports* to your shells rc file,

```
export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
export CROSS_COMPILE=/usr/local/bin/arm-none-eabi- # macOS via MacPorts
export CROSS_COMPILE=/usr/bin/arm-none-eabi-       # Linux
```

You should be able to build the blinky example to ensure everything works.

```
cd ~/zephyr/zephyr
west build -p auto -b <your-board-name> samples/basic/blinky
```

## Devicetree details

The device tree is very basic as we will only need two buttons to interface with.
The ADC is designed to run manually outside of the Zephyr RTOS API because of the
lack of continuous mode throught the DMA channel. As shown, the ``sw0`` devicetree
alias must point to a child node of a node

## Building and Running

This sample can be built for the Nucleo STM32F767ZI in this example we will
build it for the nucleo_f767zi board:

```
cd ~/zephyr/zephyr
git clone https://github.com/nanomagneticslab/pwjrtos.git
west build -p auto -b nucleo_f767zi pwjrtos
west flash
```

You can set the debugging options in `prj.conf`.
