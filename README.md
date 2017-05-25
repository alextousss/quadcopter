# quadcopter
quadcopter management code

There are 3 majors functions actually : 

- Compute IMU sensor data (gyroscope and accelerometer) and fusion them to a single absolute orientation (in degrees)<br />
 -> it is made inside the IMUsensor class (IMUsensor.cpp / IMUsensor.hpp)<br />

- Compute command to stabilize the quadcopter, using the orientation from the IMU and the altitude from the Ultrasonic Module and a PID algorithm, <br />
 -> it is made inside the PID class (PID.cpp / PID.hpp) <br /> 
 
- Compute motors controls using the PID commands. <br />
 -> it is made inside the MotorManager class (motormanager.cpp / motormanager.hpp) <br />
