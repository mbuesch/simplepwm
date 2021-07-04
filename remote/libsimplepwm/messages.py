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
    "SimplePWMMsg",
    "SimplePWMMsg_Nop",
    "SimplePWMMsg_Ack",
    "SimplePWMMsg_Nack",
    "SimplePWMMsg_Ping",
    "SimplePWMMsg_Pong",
    "SimplePWMMsg_GetControl",
    "SimplePWMMsg_Control",
    "SimplePWMMsg_GetSetpoints",
    "SimplePWMMsg_Setpoints",
    "SimplePWMMsg_GetPwmcorr",
    "SimplePWMMsg_Pwmcorr",
    "SimplePWMMsg_GetBatvolt",
    "SimplePWMMsg_Batvolt",
    "SimplePWMMsg_Enterboot",
]

from libsimplepwm.crc import *
from libsimplepwm.util import *

class SimplePWMMsg(object):
    PAYLOAD_SIZE            = 8
    SIZE                    = 1 + 1 + 1 + PAYLOAD_SIZE + 1

    MSG_MAGIC               = 0xAA
    MSG_SYNCBYTE            = b"\x00"

    MSGID_NOP               = 0
    MSGID_ACK               = 1
    MSGID_NACK              = 2
    MSGID_PING              = 3
    MSGID_PONG              = 4
    MSGID_GET_CONTROL       = 5
    MSGID_CONTROL           = 6
    MSGID_GET_SETPOINTS     = 7
    MSGID_SETPOINTS         = 8
    MSGID_GET_PWMCORR       = 9
    MSGID_PWMCORR           = 10
    MSGID_GET_BATVOLT       = 11
    MSGID_BATVOLT           = 12
    MSGID_ENTERBOOT         = 0xFF

    @staticmethod
    def _toLe16(value):
        return (value & 0xFFFF).to_bytes(2, "little")

    @staticmethod
    def _fromLe16(data, signed=False):
        return int.from_bytes(data, "little", signed=signed)

    @classmethod
    def parse(cls, data):
        assert len(data) == cls.SIZE
        magic, msgId = data[0:2]
        payload = data[3:-1]
        crc = data[-1]
        if magic != cls.MSG_MAGIC:
            printError("Received corrupted message: Magic byte mismatch.")
        elif crc8(data[:-1]) != crc:
            printError("Received corrupted message: CRC mismatch.")
        else:
            if msgId == cls.MSGID_NOP:
                return SimplePWMMsg_Nop._parse(payload)
            elif msgId == cls.MSGID_ACK:
                return SimplePWMMsg_Ack._parse(payload)
            elif msgId == cls.MSGID_NACK:
                return SimplePWMMsg_Nack._parse(payload)
            elif msgId == cls.MSGID_PING:
                return SimplePWMMsg_Ping._parse(payload)
            elif msgId == cls.MSGID_PONG:
                return SimplePWMMsg_Pong._parse(payload)
            elif msgId == cls.MSGID_GET_CONTROL:
                return SimplePWMMsg_GetControl._parse(payload)
            elif msgId == cls.MSGID_CONTROL:
                return SimplePWMMsg_Control._parse(payload)
            elif msgId == cls.MSGID_GET_SETPOINTS:
                return SimplePWMMsg_SetSetpoints._parse(payload)
            elif msgId == cls.MSGID_SETPOINTS:
                return SimplePWMMsg_Setpoints._parse(payload)
            elif msgId == cls.MSGID_GET_PWMCORR:
                return SimplePWMMsg_GetPwmcorr._parse(payload)
            elif msgId == cls.MSGID_PWMCORR:
                return SimplePWMMsg_Pwmcorr._parse(payload)
            elif msgId == cls.MSGID_GET_BATVOLT:
                return SimplePWMMsg_GetBatvolt._parse(payload)
            elif msgId == cls.MSGID_BATVOLT:
                return SimplePWMMsg_Batvolt._parse(payload)
            elif msgId == cls.MSGID_ENTERBOOT:
                return SimplePWMMsg_Enterboot._parse(payload)
            else:
                printError(f"Received unknown message: 0x{msgId:X}")
        return None

    @classmethod
    def _parse(cls, payload):
        return cls()

    def __init__(self, msgId):
        self.msgId = msgId

    def getData(self, payload=None):
        if payload is None:
            payload = b"\x00" * self.PAYLOAD_SIZE
        if len(payload) < self.PAYLOAD_SIZE:
            payload += b"\x00" * (self.PAYLOAD_SIZE - len(payload))
        assert len(payload) == self.PAYLOAD_SIZE
        data = bytearray( (self.MSG_MAGIC, self.msgId, 0, ) ) + payload
        data.append(crc8(data))
        assert len(data) == self.SIZE
        return data

class SimplePWMMsg_Nop(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_NOP

    def __init__(self):
        super().__init__(self.MSGID)

    def __str__(self):
        return f"NOP"

class SimplePWMMsg_Ack(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_ACK

    def __init__(self):
        super().__init__(self.MSGID)

    def __str__(self):
        return f"ACK"

class SimplePWMMsg_Nack(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_NACK

    def __init__(self):
        super().__init__(self.MSGID)

    def __str__(self):
        return f"NACK"

class SimplePWMMsg_Ping(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_PING

    def __init__(self):
        super().__init__(self.MSGID)

    def __str__(self):
        return f"PING"

class SimplePWMMsg_Pong(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_PONG

    def __init__(self):
        super().__init__(self.MSGID)

    def __str__(self):
        return f"PONG"

class SimplePWMMsg_GetControl(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_GET_CONTROL

    def __init__(self):
        super().__init__(self.MSGID)

    def __str__(self):
        return f"GET_CONTROL"

class SimplePWMMsg_Control(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_CONTROL

    MSG_CTLFLG_ANADIS    = 0x01
    MSG_CTLFLG_EEPDIS    = 0x02

    @classmethod
    def _parse(cls, payload):
        flags = payload[0]
        return cls(flags)

    def __init__(self, flags):
        super().__init__(self.MSGID)
        self.flags = flags

    def getData(self):
        payload = bytearray( (self.flags, ) )
        return super().getData(payload)

    def __str__(self):
        return f"CONTROL(flags=0x{self.flags:X})"

class SimplePWMMsg_GetSetpoints(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_GET_SETPOINTS

    MSG_GETSPFLG_HSL    = 0x01

    @classmethod
    def _parse(cls, payload):
        flags = payload[0]
        return cls(flags)

    def __init__(self, flags):
        super().__init__(self.MSGID)
        self.flags = flags

    def getData(self):
        payload = bytearray( (self.flags, ) )
        return super().getData(payload)

    def __str__(self):
        return f"GET_SETPOINTS(flags=0x{self.flags:X})"

class SimplePWMMsg_Setpoints(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_SETPOINTS

    MSG_SPFLG_HSL        = 0x01

    @classmethod
    def _parse(cls, payload):
        flags = payload[0]
        nrSp = payload[1]
        if nrSp > 3:
            printError("SimplePWMMsg_Setpoints: Received invalid nr_sp.")
            return None
        setpoints = [ cls._fromLe16(payload[2 + i*2 : 2 + i*2 + 2])
                  for i in range(nrSp) ]
        return cls(flags, setpoints)

    def __init__(self, flags, setpoints):
        super().__init__(self.MSGID)
        self.flags = flags
        self.setpoints = setpoints

    def getData(self):
        payload = bytearray( (self.flags, len(self.setpoints), ) )
        for sp in self.setpoints:
            payload += self._toLe16(sp)
        return super().getData(payload)

    def __str__(self):
        return f"SETPOINTS(flags=0x{self.flags:X}, setpoints={self.setpoints})"

class SimplePWMMsg_GetPwmcorr(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_GET_PWMCORR

    @classmethod
    def _parse(cls, payload):
        index = payload[0]
        return cls(index)

    def __init__(self, index):
        super().__init__(self.MSGID)
        self.index = index

    def getData(self):
        payload = bytearray( (self.index, ) )
        return super().getData(payload)

    def __str__(self):
        return f"GET_PWMCORR(index={self.index})"

class SimplePWMMsg_Pwmcorr(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_PWMCORR

    @classmethod
    def _parse(cls, payload):
        index = payload[0]
        numerator = cls._fromLe16(payload[1 : 1 + 2])
        denominator = cls._fromLe16(payload[3 : 3 + 2])
        if numerator == denominator:
            numerator = denominator = 1
        try:
            fraction = Fraction(numerator, denominator)
        except (ValueError, ZeroDivisionError) as e:
            printError("SimplePWMMsg_Pwmcorr: Received invalid fraction.")
            return None
        return cls(index, fraction)

    def __init__(self, index, fraction):
        super().__init__(self.MSGID)
        self.index = index
        self.fraction = fraction

    def getData(self):
        payload = bytearray( (self.index,) )
        numerator, denominator = self.fraction.numerator, self.fraction.denominator
        if numerator == denominator:
            numerator = denominator = 0
        payload += self._toLe16(numerator)
        payload += self._toLe16(denominator)
        return super().getData(payload)

    def __str__(self):
        return f"PWMCORR(index={self.index}, fraction={self.fraction})"

class SimplePWMMsg_GetBatvolt(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_GET_BATVOLT

    def __init__(self):
        super().__init__(self.MSGID)

    def __str__(self):
        return f"GET_BATVOLT"

class SimplePWMMsg_Batvolt(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_BATVOLT

    @classmethod
    def _parse(cls, payload):
        meas = cls._fromLe16(payload[0:2])
        drop = cls._fromLe16(payload[2:4])
        return cls(meas, drop)

    def __init__(self, meas, drop):
        super().__init__(self.MSGID)
        self.meas = meas
        self.drop = drop

    def getData(self):
        payload = bytearray()
        payload += self._toLe16(self.meas)
        payload += self._toLe16(self.drop)
        return super().getData(payload)

    def __str__(self):
        return f"BATVOLT(meas={self.meas}, drop={self.drop})"

class SimplePWMMsg_Enterboot(SimplePWMMsg):
    MSGID = SimplePWMMsg.MSGID_ENTERBOOT

    MSG_BOOTMAGIC = 0xB007

    @classmethod
    def _parse(cls, payload):
        if cls._fromLe16(payload[0:2]) != cls.MSG_BOOTMAGIC:
            printError("SimplePWMMsg_Enterboot: Received invalid magic.")
            return None
        return cls()

    def __init__(self):
        super().__init__(self.MSGID)

    def getData(self):
        payload = bytearray()
        payload += self._toLe16(self.MSG_BOOTMAGIC)
        return super().getData(payload)

    def __str__(self):
        return f"ENTERBOOT"

# vim: ts=4 sw=4 expandtab
