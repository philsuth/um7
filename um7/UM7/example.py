import UM7

name1 = 's'
port1 = '/dev/ttyS0'
statevars = ['health', 'roll', 'pitch', 'yaw', 'xmag', 'ymag', 'zmag', 'xmagraw', 'ymagraw', 'zmagraw', 'xaccelraw', 'yaccelraw', 'zaccelraw', 'xaccel', 'yaccel', 'zaccel', 'xgyro', 'ygyro', 'zgyro']

s1 = UM7.UM7(name1, port1, statevars)

print('GET_FW_REVISION=' +     s1.get_fw_revision())
print('ZERO_GYROS ' + 'ok.' if s1.zero_gyros()      else 'failed.')
print('RESET_EKF ' + 'ok.'  if s1.reset_ekf()       else 'failed.')
#s1.reset_to_factory()
#s1.set_mag_reference()
#s1.set_home_position()

fs = ''
hs = ''
for i in statevars:
    hs += '{:>9.9s} '
    if i == 'health':
        fs += ' {0['+i+']:08b} '
    else:
        fs += '{0['+i+']:9.2f} '

c = 0
while True:
        s1.catchallsamples(0.1)
        if c % 100 == 0:
            print(hs.format(*statevars))
        print(fs.format(s1.state))
        c += 1
