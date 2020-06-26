#!/usr/bin/env python3

import sys
from fractions import Fraction

print("/* THIS IS A GENERATED FILE */\n")
for arg in sys.argv:
	if arg.startswith("-DCONF_PWMLOWSPFACT="):
		fact = arg.split("=")[1].replace('"', '').strip()
		fraction = Fraction(fact).limit_denominator(0xFFFF)
		if not 0 <= fraction.numerator <= 0xFFFF:
			raise Exception("PWM correction factor is out of range.")
		print(f"#define CONF_PWMLOWSPFACT_MUL {fraction.numerator}u")
		print(f"#define CONF_PWMLOWSPFACT_DIV {fraction.denominator}u")
