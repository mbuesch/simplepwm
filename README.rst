Simple PWM controller
=====================

`https://bues.ch/cgit/simplepwm.git/ <https://bues.ch/cgit/simplepwm.git/>`_

Simplepwm is a small PWM brightness controller for LED lamps.


Microcontroller
===============

Simplepwm supports the following microcontrollers. Not all features are supported on all microcontrollers. See the table for details.

============================  ===========  ============  =========  =========  =========  =========
..                            ATMega-328P  ATMega-88(P)  ATTiny-85  ATTiny-45  ATTiny-25  ATTiny-13
============================  ===========  ============  =========  =========  =========  =========
Number of analog pot. inputs       3            3            1          1          1          1
Number of PWM outputs              3            3            1          1          1          1
Power saving                       x                         x          x
Battery voltage monitoring         x                         x          x
HSL color model                    x            x
Remote control via UART            x            x
Settings in EEPROM                 x            x
Debugging via UART                 x            x
Firmware update via UART           x            x
============================  ===========  ============  =========  =========  =========  =========


Hardware schematics
===================

Schematics for ATMega based setups: `schematics-atmega/simplepwm-atmega.pdf <schematics-atmega/simplepwm-atmega.pdf>`_

Schematics for ATTiny based setups: `schematics-attiny/simplepwm-attiny.pdf <schematics-attiny/simplepwm-attiny.pdf>`_


Prebuilt firmware images
========================

The release archives of simplepwm contain prebuilt `.hex` files for all supported microcontrollers in the `hex` directory. These hex files can be flashed directly to the microcontroller with a any tool of your choice (e.g. Atmel Studio or avrdude).

The corresponding fuse settings can be found in the file `fuses.txt` in the same directory.


Building the firmware
=====================

The firmware build needs the following tool chain:

* Unix-like operating system
* GNU make
* AVR GCC
* AVR Binutils
* avrdude

Run the following commands to build the firmware:

.. code:: sh

	cd firmware
	make DEV=t85

Please specify the target microcontroller using the DEV variable as shown above.

Valid values are:

===============  ===========
Microcontroller  make option
===============  ===========
ATTiny 13        `DEV=t13`
ATTiny 25        `DEV=t25`
ATTiny 45        `DEV=t45`
ATTiny 85        `DEV=t85`
ATMega 88        `DEV=m88`
ATMega 88P       `DEV=m88p`
ATMega 328P      `DEV=m328p`
===============  ===========

Additional build options that can be passed to `make`:

===============  ======  ==============  ================================================================
make option      values  default         description
===============  ======  ==============  ================================================================
CONF_PWMLIM      1-100   100             Limit the maximum PWM duty cycle to this percentage.
CONF_PWMINVERT   0-1     0               Invert the PWM output signal(s).
CONF_LOWSPFACT   float   1.0             Correction factor for lower range PWM setpoint.
CONF_ADCINVERT   0-1     0               Invert the ADC input signal(s).
CONF_ADCHSL      0-1     depends on DEV  Interpret the ADC input signals as HSL setpoints instead of RGB.
FEAT_HIGHRES     0-1     depends on DEV  Higher resolution in very low PWM setpoint range.
FEAT_BATTERY     0-1     depends on DEV  Battery voltage monitoring
FEAT_POWERSAVE   0-1     depends on DEV  Power saving / deep sleep
FEAT_REMOTE      0-1     depends on DEV  Remote control via UART
FEAT_EEPROM      0-1     depends on DEV  Store settings to EEPROM
FEAT_BOOT        0-1     depends on DEV  Enable bootloader and firmware updating via UART.
===============  ======  ==============  ================================================================


Flashing the firmware via serial bootloader after build
=======================================================

Run the following commands to flash the firmware via serial bootloader to the target microcontroller.

This only works, if there's already a previous version of SimplePWM running on the chip. If the chip is blank, refer to ISP flashing below.

.. code:: sh

	cd firmware
	./flash-usb.sh /dev/ttyUSB0


Flashing the firmware via ISP adapter after build
=================================================

Run the following commands to flash the firmware via ISP (In System Programmer hardware adapter) to the target microcontroller:

.. code:: sh

	cd firmware
	make DEV=t85 print_fuses
	make DEV=t85 write_fuses
	make DEV=t85 write_mem

Or alternatively use any other of the available AVR flashing tools to program the .hex file and fuses.


License
=======

Copyright (c) 2018-2021 Michael Buesch <m@bues.ch>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
