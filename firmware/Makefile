# Project name
NAME			:= simplepwm

# Project source files
SRCS			:= main.c curve.c pwm.c
GEN_SRCS		:=

# Bootloader source files
BOOT_SRCS		:=
BOOT_GEN_SRCS		:=
BOOT_OFFSET		:=

# Project fuse bits
# 9.6 MHz, BOD 2.7 V, WDT on, SPIEN
LFUSE			:= 0x56
HFUSE			:= 0xFB
EFUSE			:=

# CPU speed, in Hz
F_CPU			:= 9600000UL

# Architecture configuration
GCC_ARCH		:= attiny13
AVRDUDE_ARCH		:= t13
FUNC_STACK_LIMIT	:=

# Programmer selection.
# Values can be:  avrisp2, mysmartusb
PROGRAMMER		:= avrisp2
AVRDUDE_SPEED		:= 10
AVRDUDE_SLOW_SPEED	:= 200

# Instrumentation
INSTRUMENT_FUNC		:=
BOOT_INSTRUMENT_FUNC	:=

# Additional compiler flags
CFLAGS			:= -Wno-conversion
LDFLAGS			:=

# Additional "clean" and "distclean" target files
CLEAN_FILES		:=
DISTCLEAN_FILES		:=


include avrmakelib.mk
