# Build configuration:

# Target device. May be one of:
#	t13	=> ATTiny13 with reduced feature set.
#	t25	=> ATTiny25 with full feature set.
#	t45	=> ATTiny45 with full feature set.
#	t85	=> ATTiny85 with full feature set.
#	m88p	=> ATMega88P with full feature set.
#	m328p	=> ATMega328P with full feature set.
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

$(info DEV=$(DEV) PWM_LIM=$(PWM_LIM) PWM_INVERT=$(PWM_INVERT) ADC_INVERT=$(ADC_INVERT))

# Project name
NAME			:= simplepwm

# Project source files
SRCS			:= \
			adc.c \
			battery.c \
			curve.c \
			filter.c \
			main.c \
			movingavg.c \
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
F_CPU			:= 9600000UL
LFUSE			:= 0x56
HFUSE			:= 0xFB
EFUSE			:=
endif
ifeq ($(DEV),t25)
# ATTiny 25
# 8 MHz, SUT 4 ms, BOD 2.7 V, WDT off, SPIEN
F_CPU			:= 8000000UL
LFUSE			:= 0xD2
HFUSE			:= 0xDD
EFUSE			:= 0xFF
endif
ifeq ($(DEV),t45)
# ATTiny 45
# 8 MHz, SUT 4 ms, BOD 2.7 V, WDT off, SPIEN
F_CPU			:= 8000000UL
LFUSE			:= 0xD2
HFUSE			:= 0xDD
EFUSE			:= 0xFF
endif
ifeq ($(DEV),t85)
# ATTiny 85
# 8 MHz, SUT 4 ms, BOD 2.7 V, WDT off, SPIEN
F_CPU			:= 8000000UL
LFUSE			:= 0xD2
HFUSE			:= 0xDD
EFUSE			:= 0xFF
endif
ifeq ($(DEV),m88p)
# ATMega 88P
# 8 MHz, SUT 4.1 ms, BOD 2.7 V, WDT off, SPIEN
F_CPU			:= 8000000UL
LFUSE			:= 0xD2
HFUSE			:= 0xDD
EFUSE			:= 0xF9
endif
ifeq ($(DEV),m328p)
# ATMega 328P
# 8 MHz, SUT 4.1 ms, BOD 2.7 V, WDT off, SPIEN
F_CPU			:= 8000000UL
LFUSE			:= 0xD2
HFUSE			:= 0xD9
EFUSE			:= 0xFD
endif

# Architecture configuration
GCC_ARCH		:= $(subst m,atmega,$(subst t,attiny,$(DEV)))
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
CLEAN_FILES		:= pwm0_isr.c pwm1_isr.c pwm2_isr.c
DISTCLEAN_FILES		:=


streq = $(and $(filter 1,$(words $2)),$(filter $1,$(firstword $2)))
deveq = $(call streq,$1,$(DEV))

ifeq ($(strip $(call deveq,t13)\
	      $(call deveq,t25)\
	      $(call deveq,t45)\
	      $(call deveq,t85)\
	      $(call deveq,m88p)\
	      $(call deveq,m328p)),)
$(error "DEV=$(DEV) is not supported.")
endif

include avrmakelib.mk

pwm.c: pwm0_isr.c pwm1_isr.c pwm2_isr.c

pwm0_isr.c: pwm_isr_template.c
	$(QUIET_SED) -e 's/%INDEX%/0/g' -e 's/%HEADER%/THIS IS A GENERATED FILE/g' $< > $@

pwm1_isr.c: pwm_isr_template.c
	$(QUIET_SED) -e 's/%INDEX%/1/g' -e 's/%HEADER%/THIS IS A GENERATED FILE/g' $< > $@

pwm2_isr.c: pwm_isr_template.c
	$(QUIET_SED) -e 's/%INDEX%/2/g' -e 's/%HEADER%/THIS IS A GENERATED FILE/g' $< > $@
