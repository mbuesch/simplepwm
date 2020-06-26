# Build configuration:

# Target device. May be one of:
#	t13	=> ATTiny13
#	t25	=> ATTiny25
#	t45	=> ATTiny45
#	t85	=> ATTiny85
#	m88	=> ATMega88
#	m88p	=> ATMega88P
#	m328p	=> ATMega328P
DEV			:= t85

ifeq ($(DEV),t13) # ATTiny 13
CONF_PWMLIM		:= 100
CONF_PWMINVERT		:= 0
CONF_PWMLOWSPFACT	:= 1.0
CONF_ADCINVERT		:= 0
CONF_ADCHSL		:= 0
FEAT_HIGHRES		:= 0
FEAT_BATTERY		:= 0
FEAT_POWERSAVE		:= 0
FEAT_REMOTE		:= 0
FEAT_EEPROM		:= 0
endif
ifeq ($(DEV),t25) # ATTiny 25
CONF_PWMLIM		:= 100
CONF_PWMINVERT		:= 0
CONF_PWMLOWSPFACT	:= 1.0
CONF_ADCINVERT		:= 0
CONF_ADCHSL		:= 0
FEAT_HIGHRES		:= 0
FEAT_BATTERY		:= 0
FEAT_POWERSAVE		:= 0
FEAT_REMOTE		:= 0
FEAT_EEPROM		:= 0
endif
ifeq ($(DEV),t45) # ATTiny 45
CONF_PWMLIM		:= 100
CONF_PWMINVERT		:= 0
CONF_PWMLOWSPFACT	:= 1.0
CONF_ADCINVERT		:= 0
CONF_ADCHSL		:= 0
FEAT_HIGHRES		:= 1
FEAT_BATTERY		:= 1
FEAT_POWERSAVE		:= 1
FEAT_REMOTE		:= 0
FEAT_EEPROM		:= 0
endif
ifeq ($(DEV),t85) # ATTiny 85
CONF_PWMLIM		:= 100
CONF_PWMINVERT		:= 0
CONF_PWMLOWSPFACT	:= 1.0
CONF_ADCINVERT		:= 0
CONF_ADCHSL		:= 0
FEAT_HIGHRES		:= 1
FEAT_BATTERY		:= 1
FEAT_POWERSAVE		:= 1
FEAT_REMOTE		:= 0
FEAT_EEPROM		:= 0
endif
ifeq ($(DEV),m88) # ATMega 88
CONF_PWMLIM		:= 100
CONF_PWMINVERT		:= 0
CONF_PWMLOWSPFACT	:= 1.0
CONF_ADCINVERT		:= 0
CONF_ADCHSL		:= 1
FEAT_HIGHRES		:= 1
FEAT_BATTERY		:= 0
FEAT_POWERSAVE		:= 1
FEAT_REMOTE		:= 1
FEAT_EEPROM		:= 1
endif
ifeq ($(DEV),m88p) # ATMega 88P
CONF_PWMLIM		:= 100
CONF_PWMINVERT		:= 0
CONF_PWMLOWSPFACT	:= 1.0
CONF_ADCINVERT		:= 0
CONF_ADCHSL		:= 1
FEAT_HIGHRES		:= 1
FEAT_BATTERY		:= 0
FEAT_POWERSAVE		:= 1
FEAT_REMOTE		:= 1
FEAT_EEPROM		:= 1
endif
ifeq ($(DEV),m328p) # ATMega 328P
CONF_PWMLIM		:= 100
CONF_PWMINVERT		:= 0
CONF_PWMLOWSPFACT	:= 1.0
CONF_ADCINVERT		:= 0
CONF_ADCHSL		:= 1
FEAT_HIGHRES		:= 1
FEAT_BATTERY		:= 1
FEAT_POWERSAVE		:= 1
FEAT_REMOTE		:= 1
FEAT_EEPROM		:= 1
endif



##############################################################################
##############################################################################
##############################################################################

$(info	DEV=$(DEV) \
	CONF_PWMLIM=$(CONF_PWMLIM) \
	CONF_PWMINVERT=$(CONF_PWMINVERT) \
	CONF_PWMLOWSPFACT=$(CONF_PWMLOWSPFACT) \
	CONF_ADCINVERT=$(CONF_ADCINVERT) \
	CONF_ADCHSL=$(CONF_ADCHSL) \
	FEAT_HIGHRES=$(FEAT_HIGHRES) \
	FEAT_BATTERY=$(FEAT_BATTERY) \
	FEAT_POWERSAVE=$(FEAT_POWERSAVE) \
	FEAT_REMOTE=$(FEAT_REMOTE) \
	FEAT_EEPROM=$(FEAT_EEPROM) \
)

# Project name
NAME			:= simplepwm

# Project source files
SRCS			:= \
			adc.c \
			battery.c \
			color.c \
			crc.c \
			curve.c \
			debug.c \
			eeprom.c \
			filter.c \
			main.c \
			movingavg.c \
			outputsp.c \
			potentiometer.c \
			pcint.c \
			pwm.c \
			remote.c \
			ring.c \
			standby.c \
			uart.c \
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
ifeq ($(DEV),m88)
# ATMega 88
# 8 MHz, SUT 4.1 ms, BOD 2.7 V, WDT off, SPIEN
F_CPU			:= 8000000UL
LFUSE			:= 0xD2
HFUSE			:= 0xDD
EFUSE			:= 0xF9
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
CFLAGS			:= -DCONF_PWMLIM="$(CONF_PWMLIM)" \
			   -DCONF_PWMINVERT="(!!($(CONF_PWMINVERT)))" \
			   -DCONF_PWMLOWSPFACT="$(CONF_PWMLOWSPFACT)" \
			   -DCONF_ADCINVERT="(!!($(CONF_ADCINVERT)))" \
			   -DCONF_ADCHSL="(!!($(CONF_ADCHSL)))" \
			   -DFEAT_HIGHRES="(!!($(FEAT_HIGHRES)))" \
			   -DFEAT_BATTERY="(!!($(FEAT_BATTERY)))" \
			   -DFEAT_POWERSAVE="(!!($(FEAT_POWERSAVE)))" \
			   -DFEAT_REMOTE="(!!($(FEAT_REMOTE)))" \
			   -DFEAT_EEPROM="(!!($(FEAT_EEPROM)))"
LDFLAGS			:=

# Additional "clean" and "distclean" target files
CLEAN_FILES		:= pwm_conf.h pwm0_isr.c pwm1_isr.c pwm2_isr.c
DISTCLEAN_FILES		:=


streq = $(and $(filter 1,$(words $2)),$(filter $1,$(firstword $2)))
deveq = $(call streq,$1,$(DEV))

ifeq ($(strip $(call deveq,t13)\
	      $(call deveq,t25)\
	      $(call deveq,t45)\
	      $(call deveq,t85)\
	      $(call deveq,m88)\
	      $(call deveq,m88p)\
	      $(call deveq,m328p)),)
$(error "DEV=$(DEV) is not supported.")
endif

include avrmakelib.mk

pwm.c: pwm_conf.h pwm0_isr.c pwm1_isr.c pwm2_isr.c

pwm_conf.h: pwm_conf.py
	$(QUIET_PYTHON3) $< $(CFLAGS) > $@

pwm0_isr.c: pwm_isr_template.c
	$(QUIET_SED) -e 's/%INDEX%/0/g' -e 's/%HEADER%/THIS IS A GENERATED FILE/g' $< > $@

pwm1_isr.c: pwm_isr_template.c
	$(QUIET_SED) -e 's/%INDEX%/1/g' -e 's/%HEADER%/THIS IS A GENERATED FILE/g' $< > $@

pwm2_isr.c: pwm_isr_template.c
	$(QUIET_SED) -e 's/%INDEX%/2/g' -e 's/%HEADER%/THIS IS A GENERATED FILE/g' $< > $@
