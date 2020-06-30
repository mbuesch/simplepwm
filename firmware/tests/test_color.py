import sys
import os
import contextlib
from subprocess import run, PIPE
from unittest import TestCase
from colorsys import hls_to_rgb

with contextlib.suppress(OSError):
	os.unlink("test_color_int.bin")
with contextlib.suppress(OSError):
	os.unlink("test_color_float.bin")
run(["cc", "-O2", "-DUSE_FLOAT=0", "-o", "test_color_int.bin", "test_color.c"])
run(["cc", "-O2", "-DUSE_FLOAT=1", "-o", "test_color_float.bin", "test_color.c"])

class Color_Test(TestCase):
	def __runtest(self, isfloat):
		def t(h, s, l):
			testbin = "test_color_float.bin" if isfloat else "test_color_int.bin"
			res = (int(x)
			       for x in run(["./" + testbin, str(h), str(s), str(l)],
			       		    stdout=PIPE).stdout.decode("UTF-8").split())
			exp = (round(x * 0xFFFF)
			       for x in hls_to_rgb(h / 0xFFFF, l / 0xFFFF, s / 0xFFFF))
			thres = 1 if isfloat else 6
			if not all( abs(a - b) <= thres for a, b in zip(res, exp) ):
				print("H = %X   S = %X   L = %X" % (h, s, l))
				print("res =", list(res), ", exp =", list(exp))
				assert 0, "Invalid RGB conversion."

		for h in range(0, 0xFFFF+1, 0x1000):
			for s in range(0, 0xFFFF+1, 0x1000):
				for l in range(0, 0xFFFF+1, 0x1000):
					t(h, s, l)

	def test_float(self):
		self.__runtest(isfloat=True)

	def test_int(self):
		self.__runtest(isfloat=False)
