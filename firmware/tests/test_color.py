import sys
import os
import contextlib
from subprocess import run, PIPE
from unittest import TestCase
from colorsys import hls_to_rgb

def cc(bin, src, *opts):
    print(f"{src} -> {bin}  {opts}")
    with contextlib.suppress(OSError):
        os.unlink(bin)
    run(["cc", "-Os", *opts, "-o", bin, src])
cc("test_color_int32.bin", "test_color.c", "-DUSE_FLOAT=0", "-DUSE_64BIT_MUL=0")
cc("test_color_int64.bin", "test_color.c", "-DUSE_FLOAT=0", "-DUSE_64BIT_MUL=1")
cc("test_color_float.bin", "test_color.c", "-DUSE_FLOAT=1")

class Color_Test(TestCase):
    def __runtest(self, mode):
        thres = 1 if mode == "float" else 6
        for h in range(0, 0xFFFF+1, 0x1000):
            for s in range(0, 0xFFFF+1, 0x1000):
                for l in range(0, 0xFFFF+1, 0x1000):
                    p = run([f"./test_color_{mode}.bin", f"{h}", f"{s}", f"{l}"],
                            stdout=PIPE)
                    res = (int(x) for x in p.stdout.decode("UTF-8").split())
                    exp = (round(x * 0xFFFF)
                           for x in hls_to_rgb(h / 0xFFFF, l / 0xFFFF, s / 0xFFFF))
                    if not all( abs(a - b) <= thres for a, b in zip(res, exp) ):
                        print("H = %X   S = %X   L = %X" % (h, s, l))
                        print("res =", list(res), ", exp =", list(exp))
                        assert 0, "Invalid RGB conversion."

    def test_float(self):
        self.__runtest(mode="float")

    def test_int32(self):
        self.__runtest(mode="int32")

    def test_int64(self):
        self.__runtest(mode="int64")

# vim: ts=4 sw=4 expandtab
