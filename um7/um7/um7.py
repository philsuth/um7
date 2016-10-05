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
import sys

try:
    monotonic = time.monotonic
except AttributeError:
    from monotonic import monotonic

GET_FW_REVISION       = 0xAA # (170)
FLASH_COMMIT          = 0xAB # (171)
RESET_TO_FACTORY      = 0xAC # (172)
ZERO_GYROS            = 0xAD # (173)
SET_HOME_POSITION     = 0xAE # (174)
SET_MAG_REFERENCE     = 0xB0 # (176)
RESET_EKF             = 0xB3 # (179)

DREG_HEALTH           = 0x55
DREG_GYRO_RAW_XY      = 0x56
DREG_GYRO_PROC_X      = 0x61
DREG_ACCEL_PROC_X     = 0x65
DREG_EULER_PHI_THETA  = 0x70
DREG_GYRO_BIAS_X      = 0x89

CREG_COM_SETTINGS     = 0x00
CREG_GYRO_TRIM_X      = 0x0C
CREG_MAG_CAL1_1       = 0x0F
CREG_MAG_BIAS_X       = 0x18

REG_HIDDEN            = 0xF000
H_CREG_GYRO_ALIGN1_1  = REG_HIDDEN | 0x31
H_CREG_ACCEL_ALIGN1_1 = REG_HIDDEN | 0x52
H_CREG_MAG_ALIGN1_1   = REG_HIDDEN | 0x73
H_CREG_MAG_REF        = REG_HIDDEN | 0x7C

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
    def __init__(self, foundpacket=False, hasdata=False, startaddress=0x0, data=None, commandfailed=True, timeout=False):
        self.foundpacket = bool(foundpacket)
        self.hasdata = bool(hasdata)
        self.startaddress = startaddress
        self.data = data
        self.commandfailed = bool(commandfailed)
        self.timeout = bool(timeout)

    def __str__(self):
        return '{}(start=0x{:x}, found={}, failed={}, timeout={}, hasdata={})'.format(self.__class__.__name__, self.startaddress, self.foundpacket, self.commandfailed, self.timeout, self.hasdata)

class UM7(object):
    """ Class that handles UM7 interfacing. Creates serial object for communication, contains functions to request specific
        data samples, catch any incoming data, check input buffer, and set various data broadcast rates. Currently only
        handles processed accel, gyro, and euler angle data.  Data is timed by OS.
    """
    baud_rates = { 9600: 0, 14400: 1, 19200: 2, 38400: 3, 57600: 4, 115200: 5, 128000: 6, 153600: 7, 230400: 8, 256000: 9, 460800: 10, 921600: 11 }

    def __init__(self, name, port, statevars, baud=115200):
        """Create new UM7 serial object.
        Defuault Baud Rate = 115200
        Initializes port, name, OS timer, and sensor state (dict)
        :param port: Virtual COM port to which the IMU is connected (str)
               name: name of object (str)
        """
        statevars[:] = [i for i in statevars]
        self.name = name
        self.t0 = monotonic()
        self.state = {}
        self.statevars = statevars
        for i in statevars:
            self.state.update({i: 0})
        try:
            self.serial = serial.Serial(port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=0.1)
        except OSError:
            print('Could not connect to UM7 %s.' % self.name)

    def __del__(self):
        if self.serial:
            self.serial.close()

    def __name__(self):
        return self.name

    def catchsample(self):
        """Function that catches and parses incoming data, and then updates the sensor's state to include new data. Old
        data in state is overwritten.

        :return: Newly obtained data, and updates internal sensor state
        """
        packet = self.readpacket()
        if not packet.foundpacket:
            return False
        sample = parsedatabatch(packet.data, packet.startaddress)
        if sample:
            self.state.update(sample)
        return sample

    def catchallsamples(self, wanted_state, timeout):
        sample = {}
        t0 = monotonic()
        all_found = False
        while monotonic() - t0 < timeout:
            packet = self.readpacket()
            if packet.foundpacket:
                newsample = parsedatabatch(packet.data, packet.startaddress)
                if newsample:
                    sample.update(newsample)
            if all (k in sample for k in wanted_state): # all vars found
                all_found = True
                break
        self.state.update(sample)
        return all_found

    def readpacket(self, timeout=0.1):
        """Scans for and partially parses new data packets. Binary data can then be sent to data parser

        :return: Parsed packet info
        """
        foundpacket = 0
        t0 = monotonic()
        while monotonic() - t0 < timeout:  # While elapsed time is less than timeout
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
            timeout = 1
        else:
            timeout = 0
            try:
                pt = bytearray(self.serial.read(size=1))[0]
                #print(bin(pt))
                hasdata = pt & 0b10000000
                isbatch = pt & 0b01000000
                numdatabytes = ((pt & 0b00111100) >> 2) * 4
                #print('numdatabytes={}'.format(numdatabytes))
                commandfailed = pt & 0b00000001
                hidden = pt & 0b00000010
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
                if hidden: startaddress |= REG_HIDDEN
                if ocs != cs:
                    print('bad checksum: {:4x} (should be: {:4x})'.format(cs, ocs))
                    raise ValueError
            except ValueError:
                hasdata = 0
                commandfailed = 0
                startaddress = 0
                data = 0
        return UM7Packet(foundpacket, hasdata, startaddress, data, commandfailed, timeout)

    def readreg(self, start, length=0, timeout=0.1):
        hidden = start & REG_HIDDEN
        sa = start & 0xFF
        pt = 0x0
        if length:
            pt = 0b01000000
            pt |= (length << 2)
        if hidden:
            pt |= 0b00000010
        ba = bytearray([ord('s'), ord('n'), ord('p'), pt, sa])
        cs = sum(ba)
        ba += struct.pack('!h', cs)
        self.serial.write(ba)
        t0 = monotonic()
        while monotonic() - t0 < timeout:  # While elapsed time is less than timeout
            packet = self.readpacket()
            if packet.startaddress == start:
                return packet
        return UM7Packet(startaddress=start, timeout=True)

    def writereg(self, start, length=0, data=None, timeout=0.1, no_read=False):
        hidden = start & REG_HIDDEN
        sa = start & 0xFF
        pt = 0x0
        if data:
            pt = 0b11000000
            pt |= (length << 2)
        if hidden:
            pt |= 0b00000010
        ba = bytearray([ord('s'), ord('n'), ord('p'), pt, sa])
        if data:
            ba += data
        cs = sum(ba)
        ba += struct.pack('!h', cs)
        self.serial.write(ba)
        if no_read:
            self.serial.flush()
            return UM7Packet(startaddress=start)
        t0 = monotonic()
        while monotonic() - t0 < timeout:  # While elapsed time is less than timeout
            packet = self.readpacket()
            if packet.startaddress == start:
                return packet
        return UM7Packet(startaddress=start, timeout=True)

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

    def flash_commit(self):
        p = self.writereg(FLASH_COMMIT)
        return (not p.commandfailed)

    def get_fw_revision(self):
        p = self.readreg(GET_FW_REVISION)
        if p.commandfailed:
            return False
        return p.data.decode()

    def set_baud_rate(self, baud):
        new_baud = self.baud_rates[baud] << 28
        p = self.readreg(CREG_COM_SETTINGS)
        if p.commandfailed:
            return False
        cr = struct.unpack('!I', p.data)[0]
        print('{:032b}'.format(cr))
        cr &= 0x0fffffff
        cr |= new_baud
        print('{:032b}'.format(cr))
        p = self.writereg(start=CREG_COM_SETTINGS, length=1, data=struct.pack('!I', new_baud), no_read=True)
        if not p:
            return False
        self.serial.baudrate = baud

def parsedatabatch(data, startaddress):
    health = 'health'
    gpx    = 'gyro_proc_x'
    gpy    = 'gyro_proc_y'
    gpz    = 'gyro_proc_z'
    gpt    = 'gyro_proc_time'
    grx    = 'gyro_raw_x'
    gry    = 'gyro_raw_y'
    grz    = 'gyro_raw_z'
    grt    = 'gyro_raw_time'
    apx    = 'accel_proc_x'
    apy    = 'accel_proc_y'
    apz    = 'accel_proc_z'
    apt    = 'accel_proc_time'
    arx    = 'accel_raw_x'
    ary    = 'accel_raw_y'
    arz    = 'accel_raw_z'
    art    = 'accel_raw_time'
    mpx    = 'mag_proc_x'
    mpy    = 'mag_proc_y'
    mpz    = 'mag_proc_z'
    mpt    = 'mag_proc_time'
    mrx    = 'mag_raw_x'
    mry    = 'mag_raw_y'
    mrz    = 'mag_raw_z'
    mrt    = 'mag_raw_time'
    r      = 'roll'
    p      = 'pitch'
    y      = 'yaw'
    rr     = 'roll_rate'
    pr     = 'pitch_rate'
    yr     = 'yaw_rate'
    et     = 'euler_time'
    temp   = 'temp'
    DD = 91.02222 # divider for degrees
    DR = 16.0     # divider for rate
    try:
        if startaddress == DREG_HEALTH:
            # (0x55,  85) Health register
            values = struct.unpack('!i', data)
            output = { health: values[0] }
        elif startaddress == DREG_GYRO_PROC_X:
            # (0x61,  97) Processed Data: gyro (deg/s) xyzt, accel (m/s²) xyzt, mag xyzt
            values = struct.unpack('!ffffffffffff', data)
            output = { gpx: values[0], gpy: values[1], gpz: values[ 2], gpt: values[ 3],
                       apx: values[4], apy: values[5], apz: values[ 6], apt: values[ 7],
                       mpx: values[8], mpy: values[9], mpz: values[10], mpt: values[11]}
        elif startaddress == DREG_GYRO_RAW_XY:
            # (0x56,  86) Raw Rate Gyro Data: gyro xyz#t, accel xyz#t, mag xyz#t, temp ct
            values=struct.unpack('!hhh2xfhhh2xfhhh2xfff', data)
            output = { grx: values[0]/DD, gry: values[1]/DD, grz: values[ 2]/DD, grt: values[ 3],
                       arx: values[4]/DD, ary: values[5]/DD, arz: values[ 6]/DD, art: values[ 7],
                       mrx: values[8],    mry: values[9],    mrz: values[10],    mrt: values[11],
                       temp: values[12]}
        elif startaddress == DREG_ACCEL_PROC_X:
            # (0x65, 101) Processed Accel Data
            output = {}
        elif startaddress == DREG_EULER_PHI_THETA:
            # (0x70, 112) Processed Euler Data:
            values=struct.unpack('!hhh2xhhh2xf', data)
            output = { r:  values[0]/DD, p:  values[1]/DD, y:  values[2]/DD,
                       rr: values[3]/DR, pr: values[4]/DR, yr: values[5]/DR, et: values[6] }
        elif startaddress == DREG_GYRO_BIAS_X:
            # (0x89, 137) gyro bias xyz
            # values=struct.unpack('!fff', data)
            output = {}
        elif startaddress == CREG_GYRO_TRIM_X:
            # (0x0C,  12)
            # values=struct.unpack('!fff', data)
            output = {}
        else:
            if data:
                print('start=0x{:4x} len={:4d}'.format(startaddress, len(data)))
            return False
    except:
        raise
        return False
    return output
