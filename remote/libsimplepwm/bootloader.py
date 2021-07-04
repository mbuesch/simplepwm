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
    "SimplePWMBoot",
]

from libsimplepwm.util import *
import intelhex
import serial

class SimplePWMBoot(object):
    BOOTCMD_EXIT                    = 0x00
    BOOTCMD_GETID                   = 0x01
    BOOTCMD_WRITEPAGE               = 0x5A
    BOOTCMD_NOP                     = 0xFF

    BOOTRESULT_OK                   = 1
    BOOTRESULT_NOTOK                = 2

    BOOT_WRITEPAGE_MAGIC            = 0x97
    BOOT_WRITEPAGE_FLG_ERASEONLY    = 0x01
    BOOT_WRITEPAGE_FLG_EEPROM       = 0x02

    chipProperties = {
        5 : { # ATMega 88
            "flashSize"     : 0x2000,
            "eepromSize"    : 0x200,
            "bootSize"      : 0x400,
            "chunkSize"     : 0x40,
        },
        6 : { # ATMega 88P
            "flashSize"     : 0x2000,
            "eepromSize"    : 0x200,
            "bootSize"      : 0x400,
            "chunkSize"     : 0x40,
        },
        7 : { # ATMega 328P
            "flashSize"     : 0x8000,
            "eepromSize"    : 0x400,
            "bootSize"      : 0x1000,
            "chunkSize"     : 0x80,
        },
    }

    def __init__(self, serialFd):
        self.__serial = serialFd
        self.__h = None

    def loadImage(self, imageFile):
        try:
            self.__h = intelhex.IntelHex(str(imageFile))
            self.__h.padding = 0xFF
        except intelhex.IntelHexError as e:
            raise SimplePWMError(f"Failed to parse hex file '{imageFile}':\n"
                         f"{str(e)}")

    def __syncConnection(self, withUserHelp=False):
        try:
            self.__serial.timeout = 0.1
            begin = time.monotonic()
            loopTime = 7.0
            nextPrint = 1.0
            if withUserHelp:
                print(f"\n*** I need your help now!\n"
                      f"*** Please reset/restart/powercycle the target device once "
                      f"and then wait for the timer to expire...",
                      end="", flush=True)
            while True:
                self.__bootCommand(self.BOOTCMD_NOP, count=0x100)
                now = time.monotonic()
                if not withUserHelp or now >= begin + loopTime:
                    break
                if now - begin >= nextPrint:
                    print(f" {round(loopTime - nextPrint)}", end="", flush=True)
                    nextPrint += 1.0
            if withUserHelp:
                print(" 0")
            while self.__serial.read():
                pass
            self.__serial.timeout = 5.0
        except serial.SerialException as e:
            raise SimplePWMError(f"Serial bus exception:\n{e}")

    def __bootCommand(self, command, count=1, replyBytes=0):
        try:
            self.__serial.write(bytearray( (command, ) * count ))
            if replyBytes > 0:
                data = self.__serial.read(replyBytes)
                if not data:
                    raise SimplePWMError(f"Command timeout.")
                return data
        except serial.SerialException as e:
            raise SimplePWMError(f"Serial bus exception:\n{e}")

    def __readChipProperties(self):
        chipId = self.__bootCommand(self.BOOTCMD_GETID, replyBytes=1)[0]
        if chipId <= 0:
            raise SimplePWMError(f"Failed to fetch chip ID.")
        prop = self.chipProperties.get(chipId, None)
        if not prop:
            raise SimplePWMError(f"Properties of chip ID={chipId} unknown.")
        return prop

    def __writeImage(self, eeprom=False):
        if self.__h is None:
            return

        try:
            self.__syncConnection()
            prop = self.__readChipProperties()
        except SimplePWMError as e:
            # Try harder.
            self.__syncConnection(withUserHelp=True)
            prop = self.__readChipProperties()

        # Calculate the image properties.
        if eeprom:
            imageSize = prop["eepromSize"]
        else:
            imageSize = prop["flashSize"] - prop["bootSize"]
        if len(self.__h) > imageSize:
            raise SimplePWMError(f"The provided image is too big.")
        imageData = memoryview(self.__h.tobinstr(size=imageSize))
        chunkSize = prop["chunkSize"]

        memtype = "EEPROM" if eeprom else "flash"
        print(f"Writing {memtype} image ...", end="", flush=True)
        for addr in range(0, imageSize, chunkSize):
            pageData = imageData[addr : addr + chunkSize]
            eraseOnly = all(d == 0xFF for d in pageData)

            flags = 0
            if eraseOnly:
                flags |= self.BOOT_WRITEPAGE_FLG_ERASEONLY
            if eeprom:
                flags |= self.BOOT_WRITEPAGE_FLG_EEPROM
            cmdData = bytearray( (
                self.BOOT_WRITEPAGE_MAGIC,
                flags,
                addr & 0xFF,
                (addr >> 8) & 0xFF,
            ) )
            if not eraseOnly:
                cmdData += pageData
            cmdData += bytearray( (crc8(cmdData), ) )

            self.__bootCommand(self.BOOTCMD_WRITEPAGE)
            try:
                self.__serial.write(cmdData)
                result = self.__serial.read(1)
            except serial.SerialException as e:
                raise SimplePWMError(f"Serial bus exception:\n{e}")
            if not result:
                raise SimplePWMError(f"BOOTCMD_WRITEPAGE timeout at 0x{addr:04X}.")
            if result[0] != self.BOOTRESULT_OK:
                raise SimplePWMError(f"BOOTCMD_WRITEPAGE error at 0x{addr:04X}. "
                             f"Result: {result[0]}")
            print(".", end="", flush=True)
        self.__bootCommand(self.BOOTCMD_EXIT)
        print("\nWriting successful.")

    def writeFlash(self):
        self.__writeImage(eeprom=False)

    def writeEeprom(self):
        self.__writeImage(eeprom=True)

# vim: ts=4 sw=4 expandtab
