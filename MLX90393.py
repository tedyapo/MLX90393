#!/usr/bin/python -u
#
# MLX90393 Driver (through Bus Pirate)
#
# Copyright (C) 2016 Ted Yapo
#
# This code is licensed under the MIT License (see file)
#

#
# assumes MLX90393 connected to SPI port of Bus Pirate
#

import serial, time, struct, sys

class BusPirate(object):
  def __init__(self, port, baud, timeout):
    self.port = port
    self.baud = baud
    self.timeout = timeout

  def AssertCS(self):
    self.ser.write(b'\x02')
    resp = self.ser.read(1)
    if ord(resp) != 1:
      print 'AssertCS error'
      print ord(resp)
      sys.exit()

  def DeAssertCS(self):
    self.ser.write(b'\x03')
    resp = self.ser.read(1)
    if ord(resp) != 1:
      print 'DeAssertCS error'
      print ord(resp), resp
      sys.exit()

  def SetSpeed_1MHz(self):
    self.ser.write(b'\xc3')
    resp = self.ser.read(1)

  def ConfigSPI(self):
    self.ser.write(b'\x8d')
    resp = self.ser.read(1)
    if ord(resp) != 1 :
      print 'ConfigSPI error'
      print resp
      sys.exit()

  def ConfigPeripherals(self):
    self.ser.write(b'\x49')
    resp = self.ser.read(1)
    if ord(resp) != 1 :
      print 'ConfigPeripherals error'
      print resp
      sys.exit()

  def Connect(self):
    self.ser = serial.Serial(self.port,
                             self.baud,
                             timeout = self.timeout)
    resp = None
    for i in range(0, 20):
      self.ser.write(b'\x00')
      resp = self.ser.read(5)
      if resp == 'BBIO1':
        break

    if resp is None:
      print "Can't enter binary bitbang mode\n"
      sys.exit()

    # enter binary SPI mode
    self.ser.write(b'\x01')
    resp = self.ser.read(5)
    if resp != 'SPI1':
      print "can't enter binary SPI mode\n"
      print resp
      sys.exit()

    self.ConfigSPI()
    self.ConfigPeripherals()
    self.SetSpeed_1MHz()
    self.DeAssertCS()

  def BulkTransfer(self, data):
    l = len(data)
    if l == 0:
      return
    if l > 16:
      print "Error: bulk transfer with more than 16 bytes"
      sys.exit()
    self.AssertCS()
    self.ser.write(chr(0x10 | (l - 1)))
    self.ser.write(data)
    resp = self.ser.read(l + 1)
    self.DeAssertCS()
    #print 'resp = ', [hex(ord(c)) for c in resp]
    if resp[0] != b'\x01':
      print "Invalid response '", resp[0], "' received in BulkTransfer"
      sys.exit()
    return resp[1:]



class MLX90393(object):
  __slots__ = ('BP', '_GAIN_SEL', '_RES_X', '_RES_Y', '_RES_Z', 
               '_OSR', '_DIG_FLT', '_TCMP_EN',
               '_SensitivityXY', '_SensitivityZ', '_Tconv', '_StatusBits',
               '_GAIN_SEL_REG', '_GAIN_SEL_MASK', '_GAIN_SEL_SHIFT',
               '_RES_X_REG', '_RES_X_MASK', '_RES_X_SHIFT',
               '_RES_Y_REG', '_RES_Y_MASK', '_RES_Y_SHIFT',
               '_RES_Z_REG', '_RES_Z_MASK', '_RES_Z_SHIFT',
               '_OSR_REG', '_OSR_MASK', '_OSR_SHIFT',
               '_DIG_FLT_REG', '_DIG_FLT_MASK', '_DIG_FLT_SHIFT',
               '_TCMP_EN_REG', '_TCMP_EN_MASK', '_TCMP_EN_SHIFT')

  def __init__(self, BusPirate):
    self.BP = BusPirate
    self._GAIN_SEL = 0
    self._RES_X = 0
    self._RES_Y = 0
    self._RES_Z = 0
    self._OSR = 0
    self._DIG_FLT = 0
    self._TCMP_EN = 0

    self._GAIN_SEL_REG = 0x00
    self._GAIN_SEL_MASK = 0x0070
    self._GAIN_SEL_SHIFT = 4

    self._RES_Z_REG = 0x02
    self._RES_Z_MASK = 0x0600
    self._RES_Z_SHIFT = 9

    self._RES_Y_REG = 0x02
    self._RES_Y_MASK = 0x0180
    self._RES_Y_SHIFT = 7

    self._RES_X_REG = 0x02
    self._RES_X_MASK = 0x0060
    self._RES_X_SHIFT = 5

    self._OSR_REG = 0x02
    self._OSR_MASK = 0x0003
    self._OSR_SHIFT = 0

    self._DIG_FLT_REG = 0x02
    self._DIG_FLT_MASK = 0x001C
    self._DIG_FLT_SHIFT = 2

    self._TCMP_EN_REG = 0x1
    self._TCMP_EN_MASK = 0x0400
    self._TCMP_EN_SHIFT = 10

    self._StatusBits = { 'BURST_MODE' : 0x80,
                         'WOC_MDE'    : 0x40,
                         'SM_MODE'    : 0x20,
                         'ERROR'      : 0x10,
                         'SED'        : 0x08,
                         'RS'         : 0x04,
                         'D1'         : 0x02,
                         'D0'         : 0x01 }

    # X, Y sens. in uT/LSB. index with [GAIN_SEL][RES_X|Y]
    self._SensitivityXY = [[0.805, 1.610, 3.220, 6.440],
                           [0.644, 1.288, 2.576, 5.152],
                           [0.483, 0.966, 1.932, 3.864],
                           [0.403, 0.805, 1.610, 3.220],
                           [0.322, 0.644, 1.288, 2.576],
                           [0.268, 0.537, 1.073, 2.147],
                           [0.215, 0.429, 0.859, 1.717],
                           [0.161, 0.322, 0.644, 1.288]];

    # Z sens. in uT/LSB. index with [GAIN_SEL][RES_Z]
    self._SensitivityZ = [[1.468, 2.936, 5.872, 11.744],
                          [1.174, 2.349, 4.698, 9.395],
                          [0.881, 1.762, 3.523, 7.046],
                          [0.734, 1.468, 2.936, 5.872],
                          [0.587, 1.174, 2.349, 4.698],
                          [0.489, 0.979, 1.957, 3.915],
                          [0.391, 0.783, 1.566, 3.132],
                          [0.294, 0.587, 1.174, 2.349]];

    # conversion time(ms) indexed by [DIG_FLT][OSR]
    self._Tconv = [[1.27, 1.84, 3.00, 5.30],
                   [1.46, 2.23, 3.76, 6.84],
                   [1.84, 3.00, 5.30, 9.91],
                   [2.61, 4.53, 8.37, 16.05],
                   [4.15, 7.60, 14.52, 28.43],
                   [7.22, 13.75, 26.80, 52.92],
                   [13.36, 26.04, 51.38, 102.07],
                   [25.65, 50.61, 100.53, 200.37]];

  def _DumpStatus(self, status):
    for name, mask in sorted(self._StatusBits.iteritems(),
                             key=lambda x: x[1],
                             reverse = True):
      if status & mask:
        print name, ' ' * (10-len(name)), ': 1'
      else:
        print name, ' ' * (10-len(name)), ': 0'

  def _CheckStatus(self, status):
    if status & self._StatusBits['ERROR']:
      print 'Error bit reported by MLX90393'
      self._DumpStatus(status)
      sys.exit()

  def ReadRawXYZ(self, conv):
    flags = 0x0e
    count = 3
    resp = self.BP.BulkTransfer(chr(0x30 | flags) + b'\x00')
    # wait for conversion time (25% margin added)
    time.sleep(1.25 * self._Tconv[self._DIG_FLT][self._OSR] * count / 1000.)
    resp = self.BP.BulkTransfer(chr(0x40 | flags) +
                                b'\x00' + b'\x00\x00' * count)
    data = struct.unpack('>xB' + conv, resp)
    status = data[0]
    self._CheckStatus(status)
    raw = data[1:]
    return raw

  def ReadTemperature(self):
    self._CheckSettings()
    raw = self.ReadRaw('t')
    return 25 + ((raw[0]-46244) / 45.2)

  def ReadXYZ(self):
    self._CheckSettings()

    if self._TCMP_EN:
      print "TCMP_EN = 1 not yet supported"
      sys.exit()

    if self._RES_X in [2, 3]:
      conv = 'H'
    else:
      conv = 'h'
    if self._RES_Y in [2, 3]:
      conv += 'H'
    else:
      conv += 'h'
    if self._RES_Z in [2, 3]:
      conv += 'H'
    else:
      conv += 'h'

    raw = self.ReadRawXYZ(conv)

    if self._RES_X == 2:
      x = ((raw[0] - float(2**15)) *
           self._SensitivityXY[self._GAIN_SEL][self._RES_X])
    else:
      if self._RES_X == 3:
        x = ((raw[0] - float(2**14)) *
             self._SensitivityXY[self._GAIN_SEL][self._RES_X])
      else:
        x = raw[0] * self._SensitivityXY[self._GAIN_SEL][self._RES_X]

    if self._RES_Y == 2:
      y = ((raw[1] - float(2**15)) *
           self._SensitivityXY[self._GAIN_SEL][self._RES_Y])
    else:
      if self._RES_Y == 3:
        y = ((raw[1] - float(2**14)) *
             self._SensitivityXY[self._GAIN_SEL][self._RES_Y])
      else:
        y = raw[1] * self._SensitivityXY[self._GAIN_SEL][self._RES_Y]

    if self._RES_Z == 2:
      z = ((raw[2] - float(2**15)) *
           self._SensitivityZ[self._GAIN_SEL][self._RES_Z])
    else:
      if self._RES_Z == 3:
        z = ((raw[2] - float(2**14)) *
             self._SensitivityZ[self._GAIN_SEL][self._RES_Z])
      else:
        z = raw[2] * self._SensitivityZ[self._GAIN_SEL][self._RES_Z]


    return (x, y, z)

  def ReadRegister(self, address):
    cmd = b'\x50' + chr((address & 0x3f) << 2) + b'\x00\x00\x00'
    resp = self.BP.BulkTransfer(cmd)
    unpacked = struct.unpack('>xxBH', resp)
    status = unpacked[0]
    self._CheckStatus(status)
    return unpacked[1]

  def WriteRegister(self, address, data):
    cmd = b'\x60' + ( chr((data & 0xff00) >> 8) +
                      chr(data & 0xff) +
                      chr((address & 0x3f) << 2) + b'\x00' )
    resp = self.BP.BulkTransfer(cmd)
    status = struct.unpack('>xxxxB', resp)[0]
    self._CheckStatus(status)

  def Reset(self):
    resp = self.BP.BulkTransfer(b'\xf0\x00')
    status = struct.unpack('>xB', resp)[0]
    if (status & 0xfc) != 4:
      print "Reset returned bad status :", hex(status)
      sys.exit()

  # check for settings combinations forbidden by datasheet
  def _CheckSettings(self):
    if self._OSR == 0 and self._DIG_FLT == 0:
      print "setting self._OSR == 0 and self._DIG_FLT == 0 forbidden"
      sys.exit()
    if self._OSR == 0 and self._DIG_FLT == 1:
      print "setting self._OSR == 0 and self._DIG_FLT == 1 forbidden"
      sys.exit()
    if self._OSR == 1 and self._DIG_FLT == 1:
      print "setting self._OSR == 1 and self._DIG_FLT == 1 forbidden"
      sys.exit()

  @property
  def GAIN_SEL(self):
    self._GAIN_SEL = (self.ReadRegister(self._GAIN_SEL_REG) &
                      self._GAIN_SEL_MASK) >> self._GAIN_SEL_SHIFT
    return self._GAIN_SEL

  @GAIN_SEL.setter
  def GAIN_SEL(self, value):
    self._GAIN_SEL = value
    reg = self.ReadRegister(self._GAIN_SEL_REG)
    reg = ((reg & ~self._GAIN_SEL_MASK) |
           ((int(value) << self._GAIN_SEL_SHIFT) & self._GAIN_SEL_MASK))
    self.WriteRegister(self._GAIN_SEL_REG, reg)

  @property
  def RES_X(self):
    self._RES_X = (self.ReadRegister(self._RES_X_REG) &
                   self._RES_X_MASK) >> self._RES_X_SHIFT
    return self._RES_X

  @RES_X.setter
  def RES_X(self, value):
    self._RES_X = value
    reg = self.ReadRegister(self._RES_X_REG)
    reg = ((reg & ~self._RES_X_MASK) |
           ((int(value) << self._RES_X_SHIFT) & self._RES_X_MASK))
    self.WriteRegister(self._RES_X_REG, reg)

  @property
  def RES_Y(self):
    self._RES_Y = (self.ReadRegister(self._RES_Y_REG) &
                   self._RES_Y_MASK) >> self._RES_Y_SHIFT
    return self._RES_Y

  @RES_Y.setter
  def RES_Y(self, value):
    self._RES_Y = value
    reg = self.ReadRegister(self._RES_Y_REG)
    reg = ((reg & ~self._RES_Y_MASK) |
           ((int(value) << self._RES_Y_SHIFT) & self._RES_Y_MASK))
    self.WriteRegister(self._RES_Y_REG, reg)

  @property
  def RES_Z(self):
    self._RES_Z = (self.ReadRegister(self._RES_Z_REG) &
                   self._RES_Z_MASK) >> self._RES_Z_SHIFT
    return self._RES_Z

  @RES_Z.setter
  def RES_Z(self, value):
    self._RES_Z = value
    reg = self.ReadRegister(self._RES_Z_REG)
    reg = ((reg & ~self._RES_Z_MASK) |
           ((int(value) << self._RES_Z_SHIFT) & self._RES_Z_MASK))
    self.WriteRegister(self._RES_Z_REG, reg)

  @property
  def OSR(self):
    self._OSR = (self.ReadRegister(self._OSR_REG) &
                      self._OSR_MASK) >> self._OSR_SHIFT
    return self._OSR

  @OSR.setter
  def OSR(self, value):
    self._OSR = value
    reg = self.ReadRegister(self._OSR_REG)
    reg = ((reg & ~self._OSR_MASK) |
           ((int(value) << self._OSR_SHIFT) & self._OSR_MASK))
    self.WriteRegister(self._OSR_REG, reg)

  @property
  def DIG_FLT(self):
    self._DIG_FLT = (self.ReadRegister(self._DIG_FLT_REG) &
                     self._DIG_FLT_MASK) >> self._DIG_FLT_SHIFT
    return self._DIG_FLT

  @DIG_FLT.setter
  def DIG_FLT(self, value):
    self._DIG_FLT = value
    reg = self.ReadRegister(self._DIG_FLT_REG)
    reg = ((reg & ~self._DIG_FLT_MASK) |
           ((int(value) << self._DIG_FLT_SHIFT) & self._DIG_FLT_MASK))
    self.WriteRegister(self._DIG_FLT_REG, reg)

  @property
  def TCMP_EN(self):
    self._TCMP_EN = (self.ReadRegister(self._TCMP_EN_REG) &
                     self._TCMP_EN_MASK) >> self._TCMP_EN_SHIFT
    return self._TCMP_EN

  @TCMP_EN.setter
  def TCMP_EN(self, value):
    self._TCMP_EN = value
    reg = self.ReadRegister(self._TCMP_EN_REG)
    reg = ((reg & ~self._TCMP_EN_MASK) |
           ((int(value) << self._TCMP_EN_SHIFT) & self._TCMP_EN_MASK))
    self.WriteRegister(self._TCMP_EN_REG, reg)

def main():
  BP = BusPirate(port = '/dev/ttyUSB1',
                 baud = 115200,
                 timeout = 0.01);
  BP.Connect()
  MLX = MLX90393(BP)
  MLX.Reset()

  MLX.GAIN_SEL = 7
  MLX.RES_X = 0
  MLX.RES_Y = 0
  MLX.RES_Z = 0
  MLX.OSR = 3
  MLX.DIG_FLT = 7
  MLX.TCMP_EN = 0

  while (True):
    xyz = MLX.ReadXYZ()
    print '%8.2f %8.2f %8.2f' % (xyz[0], xyz[1], xyz[2])


#ser.write(b'\x40')
#ser.write(b'\x00')
#ser.write(b'\x0f')

if __name__ == '__main__':
  main()



