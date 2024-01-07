

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "imu_config.h"

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup()
{
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(imu_raw_pub);
  nh.advertise(mag_data_pub);

  bno.begin();


  delay(1000);

  bno.setExtCrystalUse(true);
  while (!nh.connected())
  {
      nh.spinOnce();
  }
  nh.loginfo("IMU CONNECTED");
  delay(1000);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{
  uint32_t t=millis();
  if ((t-tTime[0]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
  publishmag_data();
  publishimu_raw();
  tTime[0]=t;
  }
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  nh.spinOnce();
  
}


void publishimu_raw()
{
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); 
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Quaternion quat=bno.getQuat();
  imu_raw_msg.header.stamp=rosNow();
  imu_raw_msg.header.frame_id="base_link";
  imu_raw_msg.header.seq=seq;
  imu_raw_msg.orientation.w=quat.w();
  imu_raw_msg.orientation.x=quat.x();
  imu_raw_msg.orientation.y=quat.y();
  imu_raw_msg.orientation.z=quat.z();
  
  imu_raw_msg.orientation_covariance[0] = 0.0025;
  imu_raw_msg.orientation_covariance[1] = 0;
  imu_raw_msg.orientation_covariance[2] = 0;
  imu_raw_msg.orientation_covariance[3] = 0;
  imu_raw_msg.orientation_covariance[4] = 0.0025;
  imu_raw_msg.orientation_covariance[5] = 0;
  imu_raw_msg.orientation_covariance[6] = 0;
  imu_raw_msg.orientation_covariance[7] = 0;
  imu_raw_msg.orientation_covariance[8] = 0.0025;

  
  imu_raw_msg.angular_velocity.x=gyroscope.x();
  imu_raw_msg.angular_velocity.y=gyroscope.y();
  imu_raw_msg.angular_velocity.z=(gyroscope.z()*M_PI)/180; //60
  
  imu_raw_msg.angular_velocity_covariance[0] = 0.02;
  imu_raw_msg.angular_velocity_covariance[1] = 0;
  imu_raw_msg.angular_velocity_covariance[2] = 0;
  imu_raw_msg.angular_velocity_covariance[3] = 0;
  imu_raw_msg.angular_velocity_covariance[4] = 0.02;
  imu_raw_msg.angular_velocity_covariance[5] = 0;
  imu_raw_msg.angular_velocity_covariance[6] = 0;
  imu_raw_msg.angular_velocity_covariance[7] = 0;
  imu_raw_msg.angular_velocity_covariance[8] = 0.02;

  
  imu_raw_msg.linear_acceleration.x=linearaccel.x();
  imu_raw_msg.linear_acceleration.y=linearaccel.y();
  imu_raw_msg.linear_acceleration.z=linearaccel.z();
  
  imu_raw_msg.linear_acceleration_covariance[0] = 0.04;
  imu_raw_msg.linear_acceleration_covariance[1] = 0;
  imu_raw_msg.linear_acceleration_covariance[2] = 0;
  imu_raw_msg.linear_acceleration_covariance[3] = 0;
  imu_raw_msg.linear_acceleration_covariance[4] = 0.04;
  imu_raw_msg.linear_acceleration_covariance[5] = 0;
  imu_raw_msg.linear_acceleration_covariance[6] = 0;
  imu_raw_msg.linear_acceleration_covariance[7] = 0;
  imu_raw_msg.linear_acceleration_covariance[8] = 0.04;
  
  imu_raw_pub.publish(&imu_raw_msg);
  
}
void publishmag_data()
{
  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mag_msg.header.stamp=rosNow();
  mag_msg.header.frame_id="imu_mag";
  mag_msg.header.seq=seq;
  mag_msg.magnetic_field.x=magnetometer.x();
  mag_msg.magnetic_field.y=magnetometer.y();
  mag_msg.magnetic_field.z=magnetometer.z();
  mag_data_pub.publish(&mag_msg);
}

ros::Time rosNow()
{
  return nh.now();
}
