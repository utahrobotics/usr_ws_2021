Build
-----

The IMU code is not built by default so that team members can build other parts
of the project without needing to install the dependencies of the IMU package.
The code can be built explicitly with `catkin_make imu_firmware_imu_interface`.

Upload
------

After building, run `catkin_make imu_firmware_imu_interface-upload` and
press the reset button on the Teensy. The Teensy will not reboot
automatically (`-n` flag), but will run the code when it is next
plugged in.

Testing
-------

See `imu_interface.cpp` for pin numbers. Required inputs are the
clock (ClkPin), data (RxData), and data ready (IMUDataReady) signals.
Other pins are for debugging. Double check all connections before
powering the IMU. The Teensy can be plugged in before or after this.

On the Jetson, start `roscore` and then use
`rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _fast_read:=True`
to get data from the Teensy. This data can be logged using `rostopic echo`.

