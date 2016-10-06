# UM7
Python library to interface with CH Robotics UM7 IMU

Provides a `class UM7` with the following methods:
* `readreg(start, length=0)` to read register values
* `writereg(start, length=0, data=None)` to write to registers
* `readpacket()` to read and parse incoming packet data from the serial port
* `catchsample()` to read and parse incoming data and update the internal `state` accoringly
* `catchallsamples(wanted_state, timeout)` wait at most `timeout` seconds for new data listed in `wanted_state`

usage example:
```python
import um7

u = um7.UM7('um7', '/dev/ttyS0', ['health', 'pitch', 'roll', 'yaw'])
print('GET_FW_REVISION=' + u.get_fw_revision())
u.zero_gyros()

u.catchallsamples(['pitch', 'roll', 'yaw'], 0.5)
print('{0[health]:010b} {0[pitch]:8.2f} {0[roll]:8.2f} {0[yaw]:8.2f}'.format(u.state))
```
