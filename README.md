# UM7
Python library to interface with CH Robotics UM7 IMU

usage example:
```python
import UM7

u = UM7.UM7('um7', '/dev/ttyS0', ['health', 'pitch', 'roll', 'yaw'])
print('GET_FW_REVISION=' + u.get_fw_revision())
u.zero_gyros()

u.catchallsamples(0.5)
print('{0[health]:010b} {0[pitch]:8.2f} {0[roll]:8.2f} {0[yaw]:8.2f}'.format(u.state))
```
