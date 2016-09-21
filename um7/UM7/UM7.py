# -*- coding: utf-8 -*-
"""Module to interface with CHR UM7 IMU

    Important Notes:
        Most functions return False if invalid data type is found
        This module does not check incoming data checksums
        Valid arguments for object.grabsample(datatype) can be found below

"""

# Till Busch <till@bux.at>
# based on
# Daniel Kurek, d'Arbeloff Lab, MIT, January, 2016
# Module that holds UM7 class
# Creates serial objects, contains functions to parse serial data

# TODO: Settings, Calibration, Matrices

import serial
import time
import binascii
import struct
import numpy
import sys

GET_FW_REVISION   = 0xAA # (170)
FLASH_COMMIT      = 0xAB # (171)
RESET_TO_FACTORY  = 0xAC # (172)
ZERO_GYROS        = 0xAD # (173)
SET_HOME_POSITION = 0xAE # (174)
SET_MAG_REFERENCE = 0xB0 # (176)
RESET_EKF         = 0xB3 # (179)

DREG_HEALTH          = 0x55
DREG_GYRO_PROC_X     = 0x61
DREG_GYRO_RAW_XY     = 0x56
DREG_ACCEL_PROC_X    = 0x65
DREG_EULER_PHI_THETA = 0x70
DREG_GYRO_BIAS_X     = 0x89
CREG_GYRO_TRIM_X     = 0x0C

HEALTH_GPS   = 0x1
HEALTH_MAG   = 0x2
HEALTH_GYRO  = 0x4
HEALTH_ACCEL = 0x8
HEALTH_ACC_N = 0x10
HEALTH_MG_N  = 0x20
HEALTH_RES6  = 0x40
HEALTH_RES7  = 0x80
HEALTH_OVF   = 0x100

class UM7Packet(object):
    def __init__(self, foundpacket, hasdata, startaddress, data, commandfailed):
        self.foundpacket = foundpacket
        self.hasdata = hasdata
        self.startaddress = startaddress
        self.data = data
        self.commandfailed = commandfailed

    def __init__(self, um7list):
        self.foundpacket, self.hasdata, self.startaddress, self.data, self.commandfailed = um7list

class UM7(object):
    """ Class that handles UM7 interfacing. Creates serial object for communication, contains functions to request specific
        data samples, catch any incoming data, check input buffer, and set various data broadcast rates. Currently only
        handles processed accel, gyro, and euler angle data.  Data is timed by OS.
    """

    def __init__(self, name, port, statevars, baud=115200):
        """Create new UM7 serial object.
        Defuault Baud Rate = 115200
        Initializes port, name, OS timer, and sensor state (dict)
        :param port: Virtual COM port to which the IMU is connected (str)
               name: name of object (str)
        """
        statevars[:] = [i for i in statevars]
        statevars = ['time'] + statevars
        self.name = name
        self.t0 = time.time()
        self.state = {}
        self.statevars = statevars
        for i in statevars:
            self.state.update({i: 0})
        try:
            self.serial = serial.Serial(port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=0.1)  # Open serial device
            self.serial.flushInput()
            self.serial.write(b'$$$')
            print('Connected to UM7 %s.' % self.name)
        except OSError:
            print('Could not connect to UM7 %s.' % self.name)

    def __del__(self):
        """Closes virtual com port
        """
        self.serial.close()
        print('%s serial device closed' % self.name)

    def __name__(self):
        return self.name

    def catchsample(self):
        """Function that catches and parses incoming data, and then updates the sensor's state to include new data. Old
        data in state is overwritten. Data is timed by OS

        :return: Newly obtained data, and updates internal sensor state
        """
        [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
        if not foundpacket:
            return False
        sample = parsedatabatch(data, startaddress)
        if sample:
            self.state.update(sample)
        return sample

    def catchallsamples(self, timeout):
        sample = {}  # Initialize empty dict for new samples
        t0 = time.time()  # Initialize timeout timer
        while time.time() - t0 < timeout:  # While elapsed time is less than timeout
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()  # Read a packet
            if foundpacket:  # If you got one
                newsample = parsedatabatch(data, startaddress)  # extract data
                if newsample:  # If it works
                    sample.update(newsample)  # Update sample with new sample
            if list(set(self.statevars)-set(sample.keys())) == ['time']:  # If we have a new data key for every
                                                                            # var in statevar minus 'time'
                break  # Then we have all new data and can move on
        if list(set(self.statevars) - set(sample.keys())) != ['time']:  # In case we timed out before we caught every var we want
            #print('Missed some vars!')
            pass
        if sample:  # If we have any new data
            sample.update({'time': time.time() - self.t0})
        #    self.updatestate(sample)  # Update the sensor state
        self.state.update(sample)
        return self.state  # Return the sample

    def readpacket(self):
        """Scans for and partially parses new data packets. Binary data can then be sent to data parser

        :return: Parsed packet info
        """
        foundpacket = 0
        t = time.time()
        while True:
            if self.serial.inWaiting() >= 3:
                byte = self.serial.read(size=1)
                if byte == b's':
                    byte2 = self.serial.read(size=1)
                    if byte2 == b'n':
                        byte3 = self.serial.read(size=1)
                        if byte3 == b'p':
                            foundpacket = 1
                            break
                else:
                    #print(byte)
                    pass
            else:
                time.sleep(0.01)
        if foundpacket == 0:
            hasdata = 0
            commandfailed = 0
            startaddress = 0
            data = 0
        else:
            try:
                pt = bytearray(self.serial.read(size=1))[0]
                #print(bin(pt))
                hasdata = pt & 0b10000000
                isbatch = pt & 0b01000000
                numdatabytes = ((pt & 0b00111100) >> 2) * 4
                #print('numdatabytes={}'.format(numdatabytes))
                commandfailed = pt & 0b00000001
                if not isbatch:
                    numdatabytes = 4

                startaddress = bytearray(self.serial.read(size=1))[0]
                #print('start={}'.format(startaddress))
                while self.serial.inWaiting() < numdatabytes:
                    pass
                if hasdata:
                    data = bytearray(self.serial.read(size=numdatabytes))
                else:
                    data = False
                cs = bytearray(self.serial.read(size=2))
                cs = struct.unpack('!h', cs)[0]
                ocs = 0
                ocs += ord('s')
                ocs += ord('n')
                ocs += ord('p')
                ocs += pt
                ocs += startaddress
                if data:
                    ocs += sum(data)
                if ocs != cs:
                    print(bin(pt))
                    print('cs={:4x}, ocs={:4x}'.format(cs, ocs))
                    print(data)
                    raise ValueError
            except ValueError:
                hasdata = 0
                commandfailed = 0
                startaddress = 0
                data = 0
        return [foundpacket, hasdata, startaddress, data, commandfailed]

    def readreg(self, start, length=0):
        pt = 0x0
        if length:
            pt = 0b01000000
            pt |= (length << 2)
        ba = bytearray([ord('s'), ord('n'), ord('p'), pt, start])
        cs = sum(ba)
        ba += struct.pack('!h', cs)
        self.serial.flushInput()
        self.serial.write(ba)
        while True:
            packet = self.readpacket()
            foundpacket, hasdata, startaddress, data, commandfailed = packet
            if startaddress == start:
                return UM7Packet(packet)
        return False

    def writereg(self, start, length=0, data=None):
        pt = 0x0
        if data:
            pt = 0b11000000
            pt |= (length << 2)
        ba = bytearray([ord('s'), ord('n'), ord('p'), pt, start])
        if data:
            ba += data
        cs = sum(ba)
        ba += struct.pack('!h', cs)
        self.serial.flushInput()
        self.serial.write(ba)
        while True:
            packet = self.readpacket()
            foundpacket, hasdata, startaddress, data, commandfailed = packet
            if startaddress == start:
                return UM7Packet(packet)
        return False

    def settimer(self, t=False):
        """Resets internal UM7 class timer

        :param t: If given, sets class timer to t.  If not, all new data is timed relative to instant that settimer()
        is called
        :return:
        """
        if t:
            self.t0 = t
        else:
            self.t0 = time.time()

    def zero_gyros(self):
        """Sends request to zero gyros and waits for confirmation from sensor

        :return: True or False based on success of request
        """
        p = self.writereg(ZERO_GYROS)
        return (not p.commandfailed)

    def reset_ekf(self):
        """Sends request to reset ekf and waits for confirmation from sensor

        :return: True or False based on success of request
        """
        p = self.writereg(RESET_EKF)
        return (not p.commandfailed)

    def reset_to_factory(self):
        p = self.writereg(RESET_TO_FACTORY)
        return (not p.commandfailed)

    def set_mag_reference(self):
        p = self.writereg(SET_MAG_REFERENCE)
        return (not p.commandfailed)

    def set_home_position(self):
        p = self.writereg(SET_HOME_POSITION)
        return (not p.commandfailed)

    def get_fw_revision(self):
        p = self.readreg(GET_FW_REVISION)
        if p.commandfailed:
            return False
        return p.data.decode()

def parsedatabatch(data, startaddress):
    xg   = 'xgyro'
    yg   = 'ygyro'
    zg   = 'zgyro'
    xa   = 'xaccel'
    ya   = 'yaccel'
    za   = 'zaccel'
    xm   = 'xmag'
    ym   = 'ymag'
    zm   = 'zmag'
    rxm  = 'xmagraw'
    rym  = 'ymagraw'
    rzm  = 'zmagraw'
    mrt  = 'magrawtime'
    r    = 'roll'
    p    = 'pitch'
    y    = 'yaw'
    rr   = 'rollrate'
    pr   = 'pitchrate'
    yr   = 'yawrate'
    rxa  = 'xaccelraw'
    rya  = 'yaccelraw'
    rza  = 'zaccelraw'
    rxg  = 'xgyroraw'
    ryg  = 'ygyroraw'
    rzg  = 'zgyroraw'
    temp = 'temp'
    try:
        if startaddress == DREG_HEALTH:
            # (0x55,  85) Health register
            values = struct.unpack('!i', data)
            output = { 'health': values[0] }
        elif startaddress == DREG_GYRO_PROC_X:
            # (0x61,  97) Processed Data: gyro (deg/s) xyzt, accel (m/s²) xyzt, mag xyzt
            values = struct.unpack('!ffffffffffff', data)
            output = { xg: values[0], yg: values[1], zg: values[2],
                       xa: values[4], ya: values[5], za: values[6],
                       xm: values[8], ym: values[9], zm: values[10]}
        elif startaddress == DREG_GYRO_RAW_XY:
            # (0x56,  86) Raw Rate Gyro Data: gyro xyz#t, accel xyz#t, mag xyz#t, temp ct
            values=struct.unpack('!hhh2xfhhh2xfhhh2xfff', data)
            output = { rxg: values[0]/91.02222, ryg: values[1]/91.02222, rzg: values[2]/91.02222,
                       rxa: values[4]/91.02222, rya: values[5]/91.02222, rza: values[6]/91.02222,
                       rxm: values[8], rym: values[9], rzm: values[10], mrt: values[11],
                       temp: values[12]}
        elif startaddress == DREG_ACCEL_PROC_X:
            # (0x65, 101) Processed Accel Data
            output = {}
        elif startaddress == DREG_EULER_PHI_THETA:
            # (0x70, 112) Processed Euler Data:
            values=struct.unpack('!hhhhhhhhf', data)
            output = { r:  values[0]/91.02222, p:  values[1]/91.02222, y:  values[2]/91.02222,
                       rr: values[4]/16.0,     pr: values[5]/16.0,     yr: values[6]/16.0 }
        elif startaddress == DREG_GYRO_BIAS_X:
            # (0x89, 137) gyro bias xyz
            values=struct.unpack('!fff', data)
            output = {}
        elif startaddress == CREG_GYRO_TRIM_X:
            # (0x0C,  12)
            values=struct.unpack('!fff', data)
            output = {}
        else:
            if data:
                print('start=0x{:4x} len={:4d}'.format(startaddress, len(data)))
            return False
    except:
        raise
        return False
    return output
