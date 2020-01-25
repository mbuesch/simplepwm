# Build configuration:

# Target device. May be one of:
#	t13	=> ATTiny13 with reduced feature set.
#	t25	=> ATTiny25 with full feature set.
#	t45	=> ATTiny45 with full feature set.
#	t85	=> ATTiny85 with full feature set.
DEV			:= t85

# Limit the maximum PWM duty cycle to this percentage:
PWM_LIM			:= 100
# Invert the PWM output signal?
PWM_INVERT		:= 0

# Invert the ADC input signal?
ADC_INVERT		:= 0



##############################################################################
##############################################################################
##############################################################################

# Project name
NAME			:= simplepwm

# Project source files
SRCS			:= \
			adc.c \
			battery.c \
			curve.c \
			filter.c \
			main.c \
			potentiometer.c \
			pwm.c \
			watchdog.c

GEN_SRCS		:=

# Bootloader source files
BOOT_SRCS		:=
BOOT_GEN_SRCS		:=
BOOT_OFFSET		:=

# Project fuse bits
ifeq ($(DEV),t13)
# ATTiny 13
# 9.6 MHz, SUT 4 ms, BOD 2.7 V, WDT on, SPIEN
LFUSE			:= 0x56
HFUSE			:= 0xFB
EFUSE			:=
else
# ATTiny 25/45/85
# 8 MHz, SUT 4 ms, BOD 2.7 V, WDT off, SPIEN
LFUSE			:= 0xD2
HFUSE			:= 0xDD
EFUSE			:= 0xFF
endif

# CPU speed, in Hz
ifeq ($(DEV),t13)
# ATTiny 13
F_CPU			:= 9600000UL
else
# ATTiny 25/45/85
F_CPU			:= 8000000UL
endif

# Architecture configuration
GCC_ARCH		:= $(subst t,attiny,$(DEV))
AVRDUDE_ARCH		:= $(DEV)
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
CFLAGS			:= -DPWM_LIM="$(PWM_LIM)" \
			   -DPWM_INVERT="(!!($(PWM_INVERT)))" \
			   -DADC_INVERT="(!!($(ADC_INVERT)))"
LDFLAGS			:=

# Additional "clean" and "distclean" target files
CLEAN_FILES		:=
DISTCLEAN_FILES		:=


streq = $(and $(filter 1,$(words $2)),$(filter $1,$(firstword $2)))
deveq = $(call streq,$1,$(DEV))
ifeq ($(strip $(call deveq,t13)$(call deveq,t25)$(call deveq,t45)$(call deveq,t85)),)
$(error "DEV=$(DEV) is not supported.")
endif

include avrmakelib.mk
