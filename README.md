# quadcopter
quadcopter control code

There are three main functions : 

- Compute IMU data (gyroscope and accelerometer) and fuse it to a single absolute orientation (in degrees)<br />
 ->made inside the IMUsensor class (IMUsensor.cpp / IMUsensor.hpp)<br />

- Compute commands to stabilize the quadcopter, using the orientation from the IMU and the altitude from the Ultrasonic Module and a PID algorithm, <br />
 -> made inside the PID class (PID.cpp / PID.hpp) <br /> 
 
- Compute motors controls using the PID commands. <br />
 -> made inside the MotorManager class (motormanager.cpp / motormanager.hpp) <br />
