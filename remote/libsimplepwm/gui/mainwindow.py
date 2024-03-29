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
    def __init__(self, connectTo=None):
        super().__init__(title="Simple PWM controller - Remote")

        self.__changeBlocked = Blocker()
        self.__comm = None

        self.set_border_width(10)

        grid = Grid()
        grid.set_column_homogeneous(True)
        self.add(grid)

        ttys = Gtk.ListStore(str)
        if connectTo:
            ttys.append([connectTo])
        for i in range(4):
            tty = f"/dev/ttyUSB{i}"
            if tty != connectTo:
                ttys.append([tty])
        for i in range(2):
            tty = f"/dev/ttyS{i}"
            if tty != connectTo:
                ttys.append([tty])
        self.__ttyCombo = Gtk.ComboBox.new_with_model_and_entry(ttys)
        self.__ttyCombo.set_entry_text_column(0)
        self.__ttyCombo.set_active(0)
        grid.attach(self.__ttyCombo, 0, 0, 1, 1)

        self.__connectButton = Gtk.Button.new_with_mnemonic("_Connect")
        grid.attach(self.__connectButton, 1, 0, 1, 1)

        self.__adjWidget = ColorAdjWidget()
        self.__adjWidget.set_sensitive(False)
        grid.attach(self.__adjWidget, 0, 1, 2, 1)

        self.__connectButton.connect("clicked", self.__on_connect)
        self.__adjWidget.connect("colorChanged", self.__on_colorChanged)
        self.connect("destroy", Gtk.main_quit)

        if connectTo:
            self.__on_connect(None)

    def __getTTY(self):
        return self.__ttyCombo.get_model().get_value(self.__ttyCombo.get_active_iter(), 0)

    def __connect(self):
        if self.__comm:
            self.__comm.disconnect()
            self.__comm = None
        try:
            self.__comm = SimplePWM(port=self.__getTTY(),
                                    dumpDataStream=False,
                                    timeout=0.1)
            r, g, b = self.__comm.getRGB()
        except SimplePWMError as e:
            self.__comm = None
            raise e
        return r, g, b

    def __on_connect(self, _):
        try:
            r, g, b = self.__connect()
            self.__adjWidget.set_sensitive(True)
        except SimplePWMError as e:
            printError(e)
            msg = Gtk.MessageDialog(parent=self,
                                    flags=Gtk.DialogFlags.MODAL,
                                    type=Gtk.MessageType.ERROR,
                                    buttons=Gtk.ButtonsType.OK,
                                    text=f"Failed to connect to {self.__getTTY()}:\n{e}")
            msg.connect("response", lambda w, r: msg.destroy())
            msg.show()
            return
        with self.__changeBlocked:
            self.__adjWidget.setRGB(round(r / 0xFFFF * 100),
                                    round(g / 0xFFFF * 100),
                                    round(b / 0xFFFF * 100))
        printInfo(f"Connected to {self.__getTTY()}.")

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
                    self.__adjWidget.set_sensitive(False)

# vim: ts=4 sw=4 expandtab
