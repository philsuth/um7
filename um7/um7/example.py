from um7 import UM7

name1 = 's'
name2 = 'sensor2'
port1 = '/dev/ttyS0'
statevars = ['health', 'roll', 'pitch', 'yaw', 'xaccel', 'yaccel', 'zaccel']
statevars = ['roll', 'pitch', 'yaw', 'xgyroraw', 'ygyroraw', 'zgyroraw', 'xmag', 'ymag', 'zmag']

sensor1 = UM7(name1, port1, statevars)
sensors = [sensor1]
for i in sensors:
    #i.factoryreset()
    #i.zerogyros()
    i.resetekf()
    i.setmagref()
    i.sethome()
    i.settimer()

while True:
    #sensor2.grabsample(['xaccel', 'yaccel', 'zaccel', 'xgyro', 'roll', 'pitch', 'yaw'])
    for i in sensors:
        #i.catchsample()
        i.catchallsamples(0.5)
        print('r= {:8.2f} p= {:8.2f} y= {:8.2f} {:8.2f} {:8.2f} {:8.2f}'.format(i.state['s roll'], i.state['s pitch'], i.state['s yaw'], i.state['s xmag'], i.state['s ymag'], i.state['s zmag']))
