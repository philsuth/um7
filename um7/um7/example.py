import um7

name1 = 's'
port1 = '/dev/ttyS0'
statevars = ['health', 'roll', 'pitch', 'yaw', 'mag_proc_x', 'mag_proc_y', 'mag_proc_z', 'mag_raw_x', 'mag_raw_y', 'mag_raw_z', 'accel_raw_x', 'accel_raw_y', 'accel_raw_z', 'accel_proc_x', 'accel_proc_y', 'accel_proc_z', 'gyro_proc_x', 'gyro_proc_y', 'gyro_proc_z', 'accel_raw_time', 'accel_proc_time', 'euler_time']

s1 = um7.UM7(name1, port1, statevars, baud=115200)

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
        fs += '{0['+i+']:9.3f} '

c = 0
sv = ['roll', 'pitch', 'yaw']
while True:
        s1.catchallsamples(sv, 1.0)
        if c % 100 == 0:
            print(hs.format(*statevars))
        print(fs.format(s1.state))
        c += 1
