"""Module to interface with CHR UM7 IMU

    Important Notes:
        Timestamps are based on OS time, not sensor's internal timer
        Sensor object's most recent data points are stored in sensor.state (dict)
        Most functions return False if invalid data type is found
        This module does not check incoming data checksums
        Valid arguments for object.grabsample(datatype) can be found below

"""

# Daniel Kurek
# d'Arbeloff Lab, MIT
# January, 2016
# Module that holds UM7 class
# Creates serial objects, contains functions to parse serial data

#####################################################################
# TODO: Broadcast Rate Settings, Optimize Data Collection
#####################################################################


import serial
import time
import binascii
import struct
import numpy
import sys


class UM7array(object):

    def __init__(self, names, ports, statevars, baud=115200):
        self.t0 = time.time()  # Reference time
        self.state = {}  # Dict that holds current state
        self.statemask = {}  # dict of NaNs to mask out old data
        self.sensors = []  # List of um7 objects in array
        for i in range(len(names)):
            svars = [j for j in statevars]
            s = UM7(names[i], ports[i], svars, baud)
            self.sensors.append(s)
        statevars[:] = [i + ' ' + j for i in names for j in statevars]
        statevars = ['time'] + statevars
        self.statevars = statevars
        self.history = numpy.zeros(len(statevars))
        #for i in statevars:
        #    self.state.update({i: float('NaN')})
        #    self.statemask.update({i: float('NaN')})

    def __del__(self):
        for i in self.sensors:
            i.serial.close()
        print('Array closed.')

    def settimer(self):
        self.t0 = time.time()

    def btstart(self):
        for i in self.sensors:
            i.btstart()

    def catchsample(self):
        for i in self.sensors:
            i.catchsample()
        self.updatestate()
        self.updatehistory()

    def catchallsamples(self, timeout=0.02):  # How do we update state/history?!?!
        for i in self.sensors:
            i.catchallsamples(timeout=timeout)
            # self.updatestate(i)
            # self.updatehistory()
        self.updatestate()
        self.updatehistory()

    # def updatestate(self, s):
        # sensorstate = {k: v for k, v in s.state.items()}
        # sensorstate.pop('time')
        # mask = {k: v for k, v in self.statemask.items()}
        # mask.update(sensorstate)
        # mask.update({'time': time.time() - self.t0})
        # self.state.update(mask)

    def updatestate(self):
        self.state.update({'time': time.time() - self.t0})  # maybe mask other sensor states to avoid oversampling
        for i in self.sensors:                              # also it lets you take more accurate time measurements
            sensorstate = {k: v for k, v in i.state.items()}
            sensorstate.pop(i.name + ' time')
            self.state.update(sensorstate)

    def updatehistory(self):
        state = numpy.array([])
        for i in self.statevars:
            state = numpy.append(state, self.state[i])
        self.history = numpy.vstack((self.history, state))

    def checkbuffer(self, numbytes):
        for i in self.sensors:
            if i.serial.inWaiting() > numbytes:
                print("flush")
                i.serial.flushInput()


class UM7(object):
    """ Class that handles UM7 interfacing. Creates serial object for communication, contains functions to request specific
        data samples, catch any incoming data, check input buffer, and set various data broadcast rates. Currently only
        handles processed accel, gyro, and euler angle data.  Data is timed by OS.
    """

    def __init__(self, name, port, statevars, baud=115200):
        """Create new UM7 serial object.
        Defuault Baud Rate = 115200
        Byte Size = 8 bits
        No Parity, 1 Stop Bit, 0 second timeout
        Initializes port, name, OS timer, and sensor state (dict)
        :param port: Virtual COM port to which the IMU is connected (str)
                name: name of object (str)
        :return: UM7 Object
        """
        statevars[:] = [name + ' ' + i for i in statevars]
        statevars = [name + ' time'] + statevars
        self.name = name
        self.t0 = time.time()
        self.state = {}
        self.statemask = {}
        self.statevars = statevars
        for i in statevars:
            self.state.update({i: 0})
            self.statemask.update({i: 0})
        try:
            self.serial = serial.Serial(port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=0.1)  # Open serial device
            self.serial.flushInput()
            self.serial.write(b'$$$')
            print('Successfully connected to %s UM7!' % self.name)
        except OSError:
            print('Could not connect to %s UM7. Is it plugged in or being used by another program?' % self.name)

    def __del__(self):
        """Closes virtual com port

        :return: None
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
        sample = parsedatabatch(data, startaddress, self.name)
        if sample:
            self.state.update(sample)
        return sample

    def catchallsamples(self, timeout):
        sample = {}  # Initialize empty dict for new samples
        t0 = time.time()  # Initialize timeout timer
        while time.time() - t0 < timeout:  # While elapsed time is less than timeout
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()  # Read a packet
            if foundpacket:  # If you got one
                newsample = parsedatabatch(data, startaddress, self.name)  # extract data
                if newsample:  # If it works
                    sample.update(newsample)  # Update sample with new sample
            if list(set(self.statevars)-set(sample.keys())) == [self.name + ' time']:  # If we have a new data key for every
                                                                            # var in statevar minus 'time'
                break  # Then we have all new data and can move on
        if list(set(self.statevars) - set(sample.keys())) != [self.name + ' time']:  # In case we timed out before we caught every var we want
            #print('Missed some vars!')
            pass
        if sample:  # If we have any new data
            sample.update({self.name + ' time': time.time() - self.t0})
        #    self.updatestate(sample)  # Update the sensor state
        self.state.update(sample)
        return self.state  # Return the sample

    def grabsample(self, datatype):
        """Function that flushes buffers and then requests and then waits for specific datatype. ONLY WORKS IF BROADCAST
        SETTINGS FOR REQUESTED DATA TYPE ARE ALREADY SET TO ZERO. Generally much slower than catchsample()

        :param datatype: 'xaccel', 'yaccel', 'zaccel', 'xgyro', 'ygyro', 'zgyro', 'rollpitch', 'yaw', 'rollpitchrate',
         and/or 'yawrate', given in list form
        :return: Newly obtained data, and updates internal sensor state
        """
        sample = {}
        for i in datatype:
            address = name2hex_reg[i]
            returnaddress = []
            self.serial.flushInput()
            self.request(i)
            while address != returnaddress:
                [foundpacket, hasdata, returnaddress, data, commandfailed] = self.readpacket()
            sample.update(parsedata(data, returnaddress, self.name))
        if sample:
            self.updatestate(sample)
        return sample

    def readpacket(self):
        """Scans for and partially parses new data packets. Binary data can then be sent to data parser

        :return: Parsed packet info
        """
        #self.serial.flushInput()
        foundpacket = 0
        count = 0
        t = time.time()
        while True:
            count += 1
            #print(self.serial.inWaiting())
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
                    print(byte)
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

    def request(self, datatype):
        """Sends data or command request to sensor.  Does not wait for any sort of response

        :param: Same as grab sample
        :return: Nothing
        """
        init = [0x73, 0x6e, 0x70, 0x00]
        address = name2hex_reg[datatype]
        decimalchecksum = 337 + address
        decimalchecksum1, decimalchecksum2 = divmod(decimalchecksum, 0x100)
        init.append(address)
        init.append(decimalchecksum1)
        init.append(decimalchecksum2)
        self.serial.write(init)

    def readreg(self, start, length=0):
        if length:
            pt = 0b01000000
            pt |= (length << 2)
        else:
            pt = 0x0
        print(bin(pt), start, length)
        ba = bytearray([ord('s'), ord('n'), ord('p'), pt, start])
        cs = sum(ba)
        ba += struct.pack('!h', cs)
        print(ba)
        print(len(ba))
        self.serial.flushInput()
        self.serial.write(ba)
        while True:
            foundpacket, hasdata, startaddress, data, commandfailed = self.readpacket()
            if startaddress == start and commandfailed == False:
                print(foundpacket, hasdata, startaddress, data, commandfailed)
                return(data)
                break

    def writereg(self, start, length=0, data=None):
        if data:
            pt = 0b11000000
            pt |= (length << 2)
        else:
            pt = 0x0
        print(bin(pt), start, length)
        ba = bytearray([ord('s'), ord('n'), ord('p'), pt, start])
        if data:
            ba += data
        cs = sum(ba)
        ba += struct.pack('!h', cs)
        print(ba)
        print(len(ba))
        self.serial.flushInput()
        self.serial.write(ba)
        while True:
            foundpacket, hasdata, startaddress, data, commandfailed = self.readpacket()
            if startaddress == start and commandfailed == False:
                break
            #print(response)

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

    def zerogyros(self):
        """Sends request to zero gyros and waits for confirmation from sensor

        :return: True or False based on success of request
        """
        print('Zeroing ' + self.name + ' gyros...')
        self.serial.write(b'F,1\n')
        timeout = time.time() + 0.5
        while time.time() < timeout:
            self.request('zerogyros')
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
            if startaddress == name2hex_reg['zerogyros'] and commandfailed == 0:
                print('Successfully zeroed gyros.')
                return True
            if self.serial.inWaiting() > 500:
                self.serial.flushInput()
        print('Could not zero gyros.')
        return False

    def resetekf(self):
        """Sends request to reset ekf and waits for confirmation from sensor

        :return: True or False based on success of request
        """
        print('Resetting ' + self.name + ' EFK...')
        self.serial.write(b'F,1\n')
        timeout = time.time() + 0.5
        while time.time() < timeout:
            self.request('resetekf')
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
            if startaddress == name2hex_reg['resetekf'] and commandfailed == 0:
                print('Successfully reset EKF.')
                return True
            if self.serial.inWaiting() > 500:
                self.serial.flushInput()
        print('Could not reset EKF.')
        return False

    def factoryreset(self):
        print('Resetting ' + self.name + ' MAG REF...')
        self.serial.write(b'F,1\n')
        timeout = time.time() + 0.5
        while time.time() < timeout:
            self.request('factoryreset')
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
            if startaddress == name2hex_reg['factoryreset'] and commandfailed == 0:
                print('Successfully reset EKF.')
                return True
            if self.serial.inWaiting() > 500:
                self.serial.flushInput()
        print('Could not reset EKF.')
        return False

    def setmagref(self):
        print('Resetting ' + self.name + ' MAG REF...')
        self.serial.write(b'F,1\n')
        timeout = time.time() + 0.5
        while time.time() < timeout:
            self.request('setmagref')
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
            if startaddress == name2hex_reg['setmagref'] and commandfailed == 0:
                print('Successfully reset EKF.')
                return True
            if self.serial.inWaiting() > 500:
                self.serial.flushInput()
        print('Could not reset EKF.')
        return False

    def sethome(self):
        print('Resetting ' + self.name + ' EFK...')
        self.serial.write(b'F,1\n')
        timeout = time.time() + 0.5
        while time.time() < timeout:
            self.request('sethome')
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
            if startaddress == name2hex_reg['sethome'] and commandfailed == 0:
                print('Successfully reset EKF.')
                return True
            if self.serial.inWaiting() > 500:
                self.serial.flushInput()
        print('Could not reset EKF.')
        return False

    def btstart(self):
        self.serial.flushInput()
        buff = 0
        while not buff:
            self.serial.write(b'F,1\n')
            buff = self.serial.inWaiting()
            print(buff)
            time.sleep(0.1)

    def updatestate(self, sample):
        sample.update({self.name + ' time': time.time() - self.t0})

        #todelete = list(set(sample.keys()).difference(self.state.keys()))
        #for i in todelete:
        #    sample.pop(i)
        mask = {k: v for k, v in self.statemask.items()}
        mask.update(sample)
        self.state.update(mask)

    def checkbuffer(self, numbytes):
        if self.serial.inWaiting() > numbytes:
            print("flush")
            self.serial.flushInput()


def parsedata(data, address, devicename):
    """Function called by class to parse binary data packets

    :param data:
    :param address:
    :param devicename:
    :return:
    """

    datatype = devicename + ' ' + dec2name_reg[address]

    if datatype == 'xgyro' or datatype == 'ygyro' or datatype == 'zgyro':
        data = struct.unpack('!f', data.decode('hex'))[0]
        output = {datatype: data}

    elif datatype == 'xaccel' or datatype == 'yaccel' or datatype == 'zaccel':
        data = struct.unpack('!f', data.decode('hex'))[0]
        output = {datatype: data}

    elif datatype == 'rollpitch':
        datasplit = [data[i:i + 4] for i in range(0, len(data), 4)]
        for j in range(len(datasplit)):
            datasplit[j] = struct.unpack('!h', datasplit[j].decode('hex'))[0] / 91.02222
        output = {'roll': datasplit[0], 'pitch': datasplit[1]}

    elif datatype == 'yaw':
        datasplit = [data[i:i + 4] for i in range(0, len(data), 4)]
        datasplit[0] = struct.unpack('!h', datasplit[0].decode('hex'))[0] / 91.02222
        output = {datatype: datasplit[0]}

    elif datatype == 'rollpitchrate':
        datasplit = [data[i:i + 4] for i in range(0, len(data), 4)]
        for j in range(len(datasplit)):
            datasplit[j] = struct.unpack('!h', datasplit[j].decode('hex'))[0] / 16.0
        output = {'rollrate': datasplit[0], 'pitchrate': datasplit[1]}

    elif datatype == 'yawrate':
        datasplit = [data[i:i + 4] for i in range(0, len(data), 4)]
        datasplit[0] = struct.unpack('!h', datasplit[0].decode('hex'))[0] / 16.0
        output = {datatype: datasplit[0]}

    else:
        return False

    return output


def parsedatabatch(data, startaddress, devicename):
    xg = devicename + ' xgyro'
    yg = devicename + ' ygyro'
    zg = devicename + ' zgyro'
    xa = devicename + ' xaccel'
    ya = devicename + ' yaccel'
    za = devicename + ' zaccel'
    xm = devicename + ' xmag'
    ym = devicename + ' ymag'
    zm = devicename + ' zmag'
    rxm = devicename + ' xmagraw'
    rym = devicename + ' ymagraw'
    rzm = devicename + ' zmagraw'
    r = devicename + ' roll'
    p = devicename + ' pitch'
    y = devicename + ' yaw'
    rr = devicename + ' rollrate'
    pr = devicename + ' pitchrate'
    yr = devicename + ' yawrate'
    rxa = devicename + ' xaccelraw'
    rya = devicename + ' yaccelraw'
    rza = devicename + ' zaccelraw'
    rxg = devicename + ' xgyroraw'
    ryg = devicename + ' ygyroraw'
    rzg = devicename + ' zgyroraw'
    temp = devicename + ' temp'
    try:
        if startaddress == 85:
            # health
            values = struct.unpack('!i', data)
            output = { devicename + ' health': values[0] }
        elif startaddress == 97:
            # Processed Gyro Data: gyro xyzt, accel xyzt, mag xyzt
            values = struct.unpack('!ffffffffffff', data)
            output = { xg: values[0], yg: values[1], zg: values[2],
                       xa: values[4], ya: values[5], za: values[6],
                       xm: values[8], ym: values[9], zm: values[10]}
        elif startaddress == 86:
            # RAW: gyro xyz#t, accel xyz#t, mag xyz#t, temp ct, checksum
            values=struct.unpack('!hhhhfhhhhfhhhhfff', data)
            output = { rxg: values[0]/91.02222, ryg: values[1]/91.02222, rzg: values[2]/91.02222,
                       rxa: values[5]/91.02222, rya: values[6]/91.02222, rza: values[7]/91.02222,
                       rxm: values[10], rym: values[11], rzm: values[12],
                       temp: values[15]}
        elif startaddress == 101:
            # Processed Accel Data:
            output = {}
        elif startaddress == 112:  # Processed Euler Data:
            values=struct.unpack('!hhhhhhhhf', data)
            output = { r:  values[0]/91.02222, p:  values[1]/91.02222, y:  values[2]/91.02222,
                       rr: values[4]/16.0,     pr: values[5]/16.0,     yr: values[6]/16.0 }
        elif startaddress == 137:
            #gyro bias xyz
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


name2hex_reg = {'health': 0x55,
               'xgyro': 0x61,
               'ygyro': 0x62,
               'zgyro': 0x63,
               'xaccel': 0x65,
               'yaccel': 0x66,
               'zaccel': 0x67,
               'rollpitch': 0x70,
               'yaw': 0x71,
               'rollpitchrate': 0x72,
               'yawrate': 0x73,
                'factoryreset': 0xAC,
                'zerogyros': 0xAD,
                'sethome': 0xAE,
                'factorycommit': 0xAF,
                'setmagref': 0xB0,
                'resetekf': 0xB3}

dec2name_reg = {85: 'health',
                97: 'xgyro',
                98: 'ygyro',
                99: 'zgyro',
                101: 'xaccel',
                102: 'yaccel',
                103: 'zaccel',
                112: 'rollpitch',
                113: 'yaw',
                114: 'rollpitchrate',
                115: 'yawrate'}
