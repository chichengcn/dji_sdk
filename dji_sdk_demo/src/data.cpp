/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_sdk_demo/data.h"
#include "dji_sdk/dji_sdk.h"

#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stdio.h>
#include <time.h>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;
ros::Publisher ground_pub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;

// serial
serial::Serial ser;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  // Publish the control signal
  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
  
  // We could use dji_sdk/flight_control_setpoint_ENUvelocity_yawrate here, but
  // we use dji_sdk/flight_control_setpoint_generic to demonstrate how to set the flag
  // properly in function Mission::step()
  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  // Publish to the ground
  ground_pub = nh.advertise<std_msgs::String>("write", 1000);


/*  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }*/

  ros::spin();
  return 0;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;

  //ROS_INFO("##### attitude_callback: %.2f, %.2f, %.2f", toEulerAngle(current_atti).x, toEulerAngle(current_atti).y, toEulerAngle(current_atti).z);
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
  ROS_INFO("##### local_position_callback: %.2f, %.2f, %.2f", current_local_pos.x, current_local_pos.y, current_local_pos.z);
}

/*WORD CalcCrc(BYTE crcbuf, WORD crc)
{
 int i;
 crc=crc ^ crcbuf;
 for(i=0;i<8;i++)
 {
  BYTE chk;
  chk=crc&1;
  crc=crc>>1;
  crc=crc&0x7fff;
  if(chk==1)
   crc=crc^0xa001;//x16+x14+1
  crc=crc&0xffff;
 }+
 return crc;
}

WORD CRCCheck16(WORD crc, BYTE *buf, int len)
{
 BYTE hi,lo;
 int i;
 
 for(i=0;i<len;i++)
 {
  crc=CalcCrc(*buf,crc);
  buf++;
 }
 hi=crc%256;
 lo=crc/256;
 crc=(lo<<8)|hi;
 return crc;
}*/

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  char buf[200];
  static int cnt = 0;
  std_msgs::String str;
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  current_gps = *msg;

  if (++cnt == 5){
    sprintf(buf, "$POS,%.8f,%.8f,%.3f\r\n", current_gps.latitude, current_gps.longitude, current_gps.altitude);
    str.data = buf;
    ground_pub.publish(str);
    cnt = 0;
  }

}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
  ROS_INFO("##### flight status: %d", flight_status);
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}



bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
