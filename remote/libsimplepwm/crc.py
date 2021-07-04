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
    "crc8",
]

from libsimplepwm.util import *

def crc8(data):
    P = 0x07
    crc = 0
    for d in data:
        tmp = crc ^ d
        for i in range(8):
            if tmp & 0x80:
                tmp = ((tmp << 1) & 0xFF) ^ P
            else:
                tmp = (tmp << 1) & 0xFF
        crc = tmp
    return crc ^ 0xFF

# vim: ts=4 sw=4 expandtab
