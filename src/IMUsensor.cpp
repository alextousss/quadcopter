#include "IMUsensor.hpp"


#define MPU_addr 0x68
#define FS_SEL 7506.0f // 131*180/pi (original FS_SEL to get degrees/s is 131)


IMUsensor::IMUsensor()
{
    raw_gyro.x = 0; //raw gyroscope data coming from the IMU
    raw_gyro.y = 0;
    raw_gyro.z = 0;

    offset_gyro.x = 0; //offset for the calibration of the gyroscope
    offset_gyro.y = 0;
    offset_gyro.z = 0;

    raw_accel.x = 0; //raw accelerometer data coming from the IMU
    raw_accel.y = 0;
    raw_accel.z = 0;

    last_loop_gyro.x = 0; //degrees per second since the last loop
    last_loop_gyro.y = 0;
    last_loop_gyro.z = 0;

    accel.x = 0; //absolute orientation from accelerometer
    accel.y = 0;
    accel.z = 0;

    gyro.x = 0; //absolute orientation from accelerometer
    gyro.y = 0;
    gyro.z = 0;

    gyro_angle_set = false;

    orientation.x = 0; //absolute orientation
    orientation.y = 0;
    orientation.z = 0;

    raw_temperature = 0;


}
bool IMUsensor::calcAbsoluteOrientation( float complementary_rate )
{

    last_loop_gyro.x = (float)raw_gyro.x / FS_SEL;
    last_loop_gyro.y = (float)raw_gyro.y / FS_SEL;
    last_loop_gyro.z = (float)raw_gyro.z / FS_SEL;

    gyro.x = last_loop_gyro.x * time_loop + orientation.x;
    gyro.y = last_loop_gyro.y * time_loop + orientation.y;
    gyro.z = last_loop_gyro.z * time_loop + orientation.z;


    accel.x = atan(raw_accel.y/sqrt(pow(raw_accel.x, 2) + pow(raw_accel.z, 2)));    //converts the raw accelerometer values to an absolute orientation (in radians)
    accel.y = atan(-1*raw_accel.x/sqrt(pow(raw_accel.y, 2) + pow(raw_accel.z, 2)));

    if(gyro_angle_set)
    {
        orientation.x = (complementary_rate * gyro.x) + ((1 - complementary_rate) * accel.x);
        orientation.y = (complementary_rate * gyro.y) + ((1 - complementary_rate) * accel.y);
        orientation.z = gyro.z;
    }
    else
    {
        orientation.x = accel.x;
        orientation.y = accel.y;
        orientation.z = 0;

        gyro_angle_set = true;
    }
    return true;
}


bool IMUsensor::actualizeSensorData()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);


    raw_accel.x=Wire.read()<<8|Wire.read();
    raw_accel.y=Wire.read()<<8|Wire.read();
    raw_accel.z=Wire.read()<<8|Wire.read();

    raw_temperature=Wire.read()<<8|Wire.read();

    raw_gyro.x=Wire.read()<<8|Wire.read();
    raw_gyro.y=Wire.read()<<8|Wire.read();
    raw_gyro.z=Wire.read()<<8|Wire.read();

    raw_gyro.x -= offset_gyro.x;
    raw_gyro.y -= offset_gyro.y;
    raw_gyro.z -= offset_gyro.z;


    time_loop = micros() / 1e6f - last_data_refresh / 1e6f;
    last_data_refresh = micros();

    return true;

}

bool IMUsensor::calibrateSensors()
{
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1C);                                                    //Send the requested starting register
    Wire.write(0x10);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1B);                                                    //Send the requested starting register
    Wire.write(0x08);                                                    //Set the requested starting register
    Wire.endTransmission();

#define NUM_SAMPLES 800

    for (int i = 0 ; i < NUM_SAMPLES ; i++) //assumes that the quadcopter has no motion during the calibration
    {
        if( i % 100 < 50 )  digitalWrite(5, HIGH); else digitalWrite(5, LOW);

        if(i > 150)
        {
            actualizeSensorData();
            offset_gyro.x += raw_accel.x;
            offset_gyro.y += raw_accel.y;
            offset_gyro.z += raw_accel.z;
        }
        delay(10);
    }

    offset_gyro.x /= NUM_SAMPLES;
    offset_gyro.y /= NUM_SAMPLES;
    offset_gyro.z /= NUM_SAMPLES;

    return true;
}
