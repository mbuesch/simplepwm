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
    "SimplePWMError",
    "printDebugDevice",
    "printDebug",
    "printInfo",
    "printWarning",
    "printError",
]

import sys

class SimplePWMError(Exception):
    pass

def printDebugDevice(msg):
    if getattr(printDebugDevice, "enabled", False):
        print("Device debug:", msg, file=sys.stderr)

def printDebug(msg):
    if getattr(printDebug, "enabled", False):
        print("Debug:", msg, file=sys.stderr)

def printInfo(msg):
    print(msg, file=sys.stderr)

def printWarning(msg):
    print("WARNING:", msg, file=sys.stderr)

def printError(msg):
    print("ERROR:", msg, file=sys.stderr)

# vim: ts=4 sw=4 expandtab
