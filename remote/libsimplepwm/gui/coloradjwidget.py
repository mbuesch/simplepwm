#
# Simple PWM controller
# Remote control tool
#
# Copyright (c) 2018-2021 Michael Buesch <m@bues.ch>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

__all__ = [
    "ColorAdjWidget",
]

from libsimplepwm.gui.blocker import *
from libsimplepwm.gui.gtk_import import *
import colorsys

def rgb2hls(r, g, b):
    h, l, s = colorsys.rgb_to_hls(r / 100.0, g / 100.0, b / 100.0)
    return h * 360.0, l * 100.0, s * 100.0

def hls2rgb(h, l, s):
    r, g, b = colorsys.hls_to_rgb(h / 360.0, l / 100.0, s / 100.0)
    return round(r * 100.0), round(g * 100.0), round(b * 100.0)

class ColorAdjWidget(Gtk.Grid):
    @GObject.Signal
    def colorChanged(self,
                     mode: str,
                     r: int, g: int, b: int,
                     h: int, l: int, s: int):
        pass

    def __init__(self):
        super().__init__()

        self.__changeBlocked = Blocker()

        self.set_column_homogeneous(True)

        label = Gtk.Label("RGB:")
        self.__scaleR = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 100, 1)
        self.__scaleG = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 100, 1)
        self.__scaleB = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 100, 1)
        self.attach(label, 0, 0, 1, 1)
        self.attach(self.__scaleR, 1, 0, 1, 1)
        self.attach(self.__scaleG, 2, 0, 1, 1)
        self.attach(self.__scaleB, 3, 0, 1, 1)

        label = Gtk.Label("HLS:")
        self.__scaleH = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 360, 1)
        self.__scaleL = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 100, 1)
        self.__scaleS = Gtk.Scale.new_with_range(Gtk.Orientation.HORIZONTAL, 0, 100, 1)
        self.attach(label, 0, 1, 1, 1)
        self.attach(self.__scaleH, 1, 1, 1, 1)
        self.attach(self.__scaleL, 2, 1, 1, 1)
        self.attach(self.__scaleS, 3, 1, 1, 1)

        self.__scaleR.connect("value_changed", self.__on_scaleRGB_changed)
        self.__scaleG.connect("value_changed", self.__on_scaleRGB_changed)
        self.__scaleB.connect("value_changed", self.__on_scaleRGB_changed)

        self.__scaleH.connect("value_changed", self.__on_scaleHLS_changed)
        self.__scaleL.connect("value_changed", self.__on_scaleHLS_changed)
        self.__scaleS.connect("value_changed", self.__on_scaleHLS_changed)

    def __on_scaleRGB_changed(self, _):
        if self.__changeBlocked:
            return
        with self.__changeBlocked:
            r = self.__scaleR.get_value()
            g = self.__scaleG.get_value()
            b = self.__scaleB.get_value()
            h, l, s = rgb2hls(r, g, b)
            self.__scaleH.set_value(h)
            self.__scaleL.set_value(l)
            self.__scaleS.set_value(s)
            self.emit("colorChanged", "RGB", r, g, b, h, l, s)

    def __on_scaleHLS_changed(self, _):
        if self.__changeBlocked:
            return
        with self.__changeBlocked:
            h = self.__scaleH.get_value()
            l = self.__scaleL.get_value()
            s = self.__scaleS.get_value()
            r, g, b = hls2rgb(h, l, s)
            self.__scaleR.set_value(r)
            self.__scaleG.set_value(g)
            self.__scaleB.set_value(b)
            self.emit("colorChanged", "HLS", r, g, b, h, l, s)

    def setRGB(self, r, g, b):
        self.__scaleR.set_value(r)
        self.__scaleG.set_value(g)
        self.__scaleB.set_value(b)

# vim: ts=4 sw=4 expandtab
