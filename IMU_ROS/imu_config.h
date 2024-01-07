
/*******************************************************************************
* Copyright 2021 Drebar
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

/*Authors: Asyraf Shahrom, Baarath Kunjunni*/

/**************************************************************
*Define ROS parameters
***************************************************************/
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

/**************************************************************
*ROS NodeHandle
***************************************************************/

ros::NodeHandle nh;
ros::Time rosNow(void);
uint32_t current_offset;

/**************************************************************
*Param definition
***************************************************************/
int seq=0;
/**************************************************************
*ROS Publisher
***************************************************************/
sensor_msgs::Imu imu_raw_msg;
ros::Publisher imu_raw_pub("imu_raw", &imu_raw_msg);

sensor_msgs::Imu imu_data_msg;
ros::Publisher imu_data_pub("imu_data", &imu_data_msg);

sensor_msgs::MagneticField  mag_msg;
ros::Publisher mag_data_pub("mag_data", &mag_msg);

/***************************************************************
*SETUP FREQUENCY
****************************************************************/
#define IMU_PUBLISH_FREQUENCY   200//Hz

/***************************************************************
* SoftwareTimer
****************************************************************/

static uint32_t tTime[10];
