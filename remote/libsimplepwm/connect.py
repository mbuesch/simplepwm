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
    "SimplePWM",
]

from collections import deque
from libsimplepwm.bootloader import *
from libsimplepwm.messages import *
from libsimplepwm.util import *
import serial
import time

class SimplePWM(object):
    FLG_8BIT                = 0x80 # 8-bit data nibble
    FLG_8BIT_UPPER          = 0x40 # 8-bit upper data nibble
    FLG_8BIT_RSV1           = 0x20 # reserved
    FLG_8BIT_RSV0           = 0x10 # reserved

    MSK_4BIT                = 0x0F # data nibble
    MSK_7BIT                = 0x7F

    REMOTE_STANDBY_DELAY_MS = 5000
    REMOTE_STANDBY_DELAY    = REMOTE_STANDBY_DELAY_MS / 1000

    def __init__(self,
                 port="/dev/ttyUSB0",
                 timeout=1.0,
                 dumpDebugStream=False,
                 recoveryMode=False):
        try:
            self.__serial = serial.Serial(port=port,
                                          baudrate=19200,
                                          bytesize=8,
                                          parity=serial.PARITY_ODD,
                                          stopbits=2,
                                          timeout=timeout)
        except serial.SerialException as e:
            raise SimplePWMError(f"Serial bus exception:\n{e}")
        self.__recoveryMode = recoveryMode
        self.__timeout = timeout
        self.__dumpDebugStream = dumpDebugStream
        self.__rxByte = 0
        self.__rxBuf = bytearray()
        self.__rxMsgs = deque()
        self.__debugBuf = bytearray()
        self.__nextSyncTime = time.monotonic()
        self.__wakeup()

    def __synchronize(self):
        self.__tx_8bit(SimplePWMMsg.MSG_SYNCBYTE * round(SimplePWMMsg.SIZE * 2.5))

    def __wakeup(self):
        nextSyncTime = self.__nextSyncTime
        self.__nextSyncTime = time.monotonic() + (self.REMOTE_STANDBY_DELAY / 2)
        if time.monotonic() < nextSyncTime:
            return
        count = 0
        while True:
            try:
                self.__synchronize()
                self.ping(0.1)
            except SimplePWMError as e:
                count += 1
                if self.__recoveryMode:
                    if count > 1:
                        break
                else:
                    if count > 10:
                        raise e
                continue
            break
        printDebug("Connection synchronized.")

    def __tx_8bit(self, data):
        self.__wakeup()
#        printDebug(f"TX: {bytes(data)}")
        for d in data:
            lo = ((d & self.MSK_4BIT) |
                  self.FLG_8BIT)
            hi = (((d >> 4) & self.MSK_4BIT) |
                  self.FLG_8BIT |
                  self.FLG_8BIT_UPPER)
            sendBytes = bytes( (lo, hi) )
            try:
                self.__serial.write(sendBytes)
            except serial.SerialException as e:
                raise SimplePWMError(f"Serial bus exception:\n{e}")

    def __rx_7bit(self, dataByte):
        self.__debugBuf += dataByte
        if dataByte == b"\n":
            if self.__dumpDebugStream:
                text = self.__debugBuf.decode("ASCII", "ignore").strip()
                printDebugDevice(text)
            self.__debugBuf.clear()

    def __rx_8bit(self, dataByte):
#        printDebug(f"RX: {dataByte}")
        d = dataByte[0]
        if d & self.FLG_8BIT_UPPER:
            data = (self.__rxByte & self.MSK_4BIT)
            data |= (d & self.MSK_4BIT) << 4
            self.__rxByte = 0
            self.__rxBuf.append(data)
            if len(self.__rxBuf) >= SimplePWMMsg.SIZE:
                rxMsg = SimplePWMMsg.parse(self.__rxBuf)
                self.__rxBuf.clear()
                if rxMsg:
                    self.__rx_message(rxMsg)
        else:
            self.__rxByte = d

    def __tx_message(self, txMsg):
        printDebug("TX msg: " + str(txMsg))
        self.__tx_8bit(txMsg.getData())

    def __rx_message(self, rxMsg):
        printDebug("RX msg: " + str(rxMsg))
        self.__rxMsgs.append(rxMsg)

    def __readNext(self, timeout):
        if timeout < 0:
            timeout = self.__timeout
        try:
            self.__serial.timeout = timeout
            dataByte = self.__serial.read(1)
        except serial.SerialException as e:
            raise SimplePWMError(f"Serial bus exception:\n{e}")
        if dataByte:
            if dataByte[0] & self.FLG_8BIT:
                self.__rx_8bit(dataByte)
            else:
                self.__rx_7bit(dataByte)

    def __waitRxMsg(self, msgType=None, timeout=None):
        if timeout is None:
            timeout = self.__timeout
        timeoutEnd = None
        if timeout >= 0.0:
            timeoutEnd = time.monotonic() + timeout
        retMsg = None
        while retMsg is None:
            if (timeoutEnd is not None and
                time.monotonic() >= timeoutEnd):
                break
            self.__readNext(timeout)
            if msgType is not None:
                for rxMsg in self.__rxMsgs:
                    if isinstance(rxMsg, msgType):
                        retMsg = rxMsg
                    else:
                        printError("Received unexpected "
                               "message: " + str(rxMsg))
            self.__rxMsgs.clear()
        return retMsg

    def dumpDebugStream(self):
        self.__dumpDebugStream = True
        while True:
            self.__waitRxMsg(timeout=-1)

    def ping(self, timeout=None):
        self.__tx_message(SimplePWMMsg_Ping())
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Pong, timeout)
        if not rxMsg:
            raise SimplePWMError("Ping failed.")

    def __getControlFlags(self, timeout=None):
        self.__tx_message(SimplePWMMsg_GetControl())
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Control, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to get control info.")
        return rxMsg.flags

    def __setControlFlags(self, flagsMask, flagsSet, timeout=None):
        self.__tx_message(SimplePWMMsg_GetControl())
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Control, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to get control info.")
        txMsg = rxMsg
        txMsg.flags = (txMsg.flags & ~flagsMask) | flagsSet
        self.__tx_message(txMsg)
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Ack, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to get acknowledge.")

    def getEepromEn(self, timeout=None):
        return not (self.__getControlFlags(timeout) &
                SimplePWMMsg_Control.MSG_CTLFLG_EEPDIS)

    def setEepromEn(self, enable, timeout=None):
        self.__setControlFlags(SimplePWMMsg_Control.MSG_CTLFLG_EEPDIS,
                       0 if enable else SimplePWMMsg_Control.MSG_CTLFLG_EEPDIS,
                       timeout)

    def getAnalogEn(self, timeout=None):
        return not (self.__getControlFlags(timeout) &
                SimplePWMMsg_Control.MSG_CTLFLG_ANADIS)

    def setAnalogEn(self, enable, timeout=None):
        self.__setControlFlags(SimplePWMMsg_Control.MSG_CTLFLG_ANADIS,
                       0 if enable else SimplePWMMsg_Control.MSG_CTLFLG_ANADIS,
                       timeout)

    def getRGB(self, timeout=None):
        self.__tx_message(SimplePWMMsg_GetSetpoints(
                flags=0))
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Setpoints, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to get RGB setpoints.")
        return rxMsg.setpoints

    def setRGB(self, rgb, timeout=None):
        self.setAnalogEn(False, timeout)
        txMsg = SimplePWMMsg_Setpoints(
                flags=0,
                setpoints=rgb)
        self.__tx_message(txMsg)
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Ack, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to set RGB setpoints.")

    def getHSL(self, timeout=None):
        self.__tx_message(SimplePWMMsg_GetSetpoints(
                flags=SimplePWMMsg_GetSetpoints.MSG_GETSPFLG_HSL))
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Setpoints, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to get HSL setpoints.")
        return rxMsg.setpoints

    def setHSL(self, hsl, timeout=None):
        self.setAnalogEn(False, timeout)
        txMsg = SimplePWMMsg_Setpoints(
                flags=SimplePWMMsg_Setpoints.MSG_SPFLG_HSL,
                setpoints=hsl)
        self.__tx_message(txMsg)
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Ack, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to set RGB setpoints.")

    def getPwmCorr(self, index, timeout=None):
        self.__tx_message(SimplePWMMsg_GetPwmcorr(index=index))
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Pwmcorr, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to get PWM correction.")
        return rxMsg.fraction

    def setPwmCorr(self, index, fraction, timeout=None):
        txMsg = SimplePWMMsg_Pwmcorr(index=index, fraction=fraction)
        self.__tx_message(txMsg)
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Ack, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to set PWM correction.")

    def getBatVoltage(self, timeout=None):
        self.__tx_message(SimplePWMMsg_GetBatvolt())
        rxMsg = self.__waitRxMsg(SimplePWMMsg_Batvolt, timeout)
        if not rxMsg:
            raise SimplePWMError("Failed to get battery voltage.")
        return (rxMsg.meas, rxMsg.drop)

    def writeFlash(self, imageFile, timeout=None):
        boot = SimplePWMBoot(self.__serial)
        boot.loadImage(imageFile)
        self.__tx_message(SimplePWMMsg_Enterboot())
        boot.writeFlash()

    def writeEeprom(self, imageFile, timeout=None):
        boot = SimplePWMBoot(self.__serial)
        boot.loadImage(imageFile)
        self.__tx_message(SimplePWMMsg_Enterboot())
        boot.writeEeprom()

# vim: ts=4 sw=4 expandtab
