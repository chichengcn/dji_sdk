
#ifndef TRAJECTIRY_H
#define TRAJECTIRY_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>
#include <stdio.h>
#include <time.h>

using namespace std;


void read_callback(const std_msgs::String::ConstPtr& msg);
void generate_traj(int type, float speed, int nstep, geometry_msgs::Twist& traj);


#endif // TRAJECTIRY_H