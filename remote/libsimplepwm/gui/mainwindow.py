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
    "MainWindow",
]

from libsimplepwm.connect import *
from libsimplepwm.gui.blocker import *
from libsimplepwm.gui.coloradjwidget import *
from libsimplepwm.gui.gtk_import import *
from libsimplepwm.util import *
import sys

class MainWindow(Gtk.Window):
    def __init__(self):
        super().__init__(title="Simple PWM controller - Remote")

        self.__changeBlocked = Blocker()
        self.__comm = None

        self.set_border_width(10)

        grid = Grid()
        grid.set_column_homogeneous(True)
        self.add(grid)

        ttys = Gtk.ListStore(str)
        for i in range(4):
            ttys.append([f"/dev/ttyUSB{i}"])
        for i in range(2):
            ttys.append([f"/dev/ttyS{i}"])
        self.ttyCombo = Gtk.ComboBox.new_with_model_and_entry(ttys)
        self.ttyCombo.set_entry_text_column(0)
        self.ttyCombo.set_active(0)
        grid.attach(self.ttyCombo, 0, 0, 1, 1)

        self.connectButton = Gtk.Button.new_with_mnemonic("_Connect")
        grid.attach(self.connectButton, 1, 0, 1, 1)

        self.adjWidget = ColorAdjWidget()
        grid.attach(self.adjWidget, 0, 1, 2, 1)

        self.connectButton.connect("clicked", self.__on_connect)
        self.adjWidget.connect("colorChanged", self.__on_colorChanged)
        self.connect("destroy", Gtk.main_quit)

    def __connect(self):
        tty = self.ttyCombo.get_model().get_value(self.ttyCombo.get_active_iter(), 0)
        try:
            self.__comm = SimplePWM(port=tty, timeout=0.1)
            r, g, b = self.__comm.getRGB()
        except SimplePWMError as e:
            self.__comm = None
            raise e
        return tty, r, g, b

    def __on_connect(self, _):
        try:
            tty, r, g, b = self.__connect()
        except SimplePWMError as e:
            printError(e)
            msg = Gtk.MessageDialog(parent=self,
                                    flags=Gtk.DialogFlags.MODAL,
                                    type=Gtk.MessageType.ERROR,
                                    buttons=Gtk.ButtonsType.OK,
                                    text=f"Failed to connect to {tty}:\n{e}")
            msg.connect("response", lambda w, r: msg.destroy())
            msg.show()
            return
        with self.__changeBlocked:
            self.adjWidget.setRGB(round(r / 0xFFFF * 100),
                                  round(g / 0xFFFF * 100),
                                  round(b / 0xFFFF * 100))
        printInfo(f"Connected to {tty}.")

    def __on_colorChanged(self, _, mode, r, g, b, h, l, s):
        if not self.__comm:
            return
        if self.__changeBlocked:
            return
        setValuesTries = 3
        while setValuesTries > 0:
            try:
                if mode == "RGB":
                    self.__comm.setRGB((round(r / 100 * 0xFFFF),
                                        round(g / 100 * 0xFFFF),
                                        round(b / 100 * 0xFFFF)))
                elif mode == "HLS":
                    self.__comm.setHSL((round(h / 360 * 0xFFFF),
                                        round(s / 100 * 0xFFFF),
                                        round(l / 100 * 0xFFFF)))
                else:
                    assert False
                setValuesTries = 0 # done
            except SimplePWMError as e:
                printError(e)
                for i in range(3):
                    try:
                        self.__connect()
                        printInfo("Reconnected.")
                        setValuesTries -= 1
                        break
                    except SimplePWMError:
                        continue
                else:
                    setValuesTries = 0 # give up

# vim: ts=4 sw=4 expandtab
