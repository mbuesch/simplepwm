import sys
from subprocess import run, PIPE
from unittest import TestCase
from colorsys import hls_to_rgb

run(["cc", "-O2", "-o", "test_color.bin", "test_color.c"])

class Color_Test(TestCase):
	def test_0(self):
		def t(h, s, l):
			res = (int(x)
			       for x in run(["./test_color.bin", str(h), str(s), str(l)],
			       		    stdout=PIPE).stdout.decode("UTF-8").split())
			exp = (round(x * 0xFFFF)
			       for x in hls_to_rgb(h / 0xFFFF, l / 0xFFFF, s / 0xFFFF))
			if not all( abs(a - b) <= 6 for a, b in zip(res, exp) ):
				print("H = %X   S = %X   L = %X" % (h, s, l))
				print("res =", list(res), ", exp =", list(exp))
				assert 0, "Invalid RGB conversion."

		for h in range(0, 0xFFFF+1, 0x1000):
			for s in range(0, 0xFFFF+1, 0x1000):
				for l in range(0, 0xFFFF+1, 0x1000):
					t(h, s, l)
