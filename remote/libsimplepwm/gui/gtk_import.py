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
    "Gtk",
    "GObject",
    "Grid",
]

import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GObject

def Grid(*args, **kwargs):
    grid = Gtk.Grid(*args, **kwargs)
    grid.set_row_spacing(10)
    grid.set_column_spacing(10)
    return grid

# vim: ts=4 sw=4 expandtab
