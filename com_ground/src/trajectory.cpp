#include <ros/ros.h>
#include "trajectory.h"

#define SPIN_RATE 10 // 10Hz
#define PI 3.14159

ros::Publisher trajPub;
ros::Publisher trajEna;

std_msgs::String gndCmd;
int trajNum = 0;
float trajSpeed = 0;

int main (int argc, char** argv){
  ros::init(argc, argv, "trajectory_node");
  ros::NodeHandle nh;

  ros::Subscriber read_sub = nh.subscribe("read", 1000, read_callback);

  // Publish the trajectory
  trajPub = nh.advertise<geometry_msgs::Twist>("desired_mission", 10);
  trajEna = nh.advertise<std_msgs::Bool>("desired_mission_enabler", 1);
  
  ros::Rate loop_rate(SPIN_RATE);
  geometry_msgs::Twist traj;
  std_msgs::Bool traj_enabler;
  int nstep = 0;
  while(ros::ok()){

    ros::spinOnce();

    if (trajNum != 0){
    	generate_traj(trajNum, trajSpeed, ++nstep, traj);
    	if (traj.angular.x)
    		trajPub.publish(traj);
    	else{
    		nstep = 0;
    		trajNum = 0;
				traj_enabler.data = false;
				trajEna.publish(traj_enabler);
    	}
    }

    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}

void read_callback(const std_msgs::String::ConstPtr& msg)
{
	gndCmd = *msg;
	std_msgs::Bool traj_enabler;
	int idx;

	if (idx = gndCmd.data.find("$CMD") != string::npos){
			traj_enabler.data = true;
			trajEna.publish(traj_enabler);
			trajNum = gndCmd.data[idx+4] - 48;
			trajSpeed = gndCmd.data[idx+6]-48 + (gndCmd.data[idx+8]-48)*0.1;
	}
}

void generate_traj(int type, float speed, int nstep, geometry_msgs::Twist& traj)
{

	switch(type){

		// Linear ahead
		case 1:{
			float dist = 5; // 5m
			if (nstep <= (dist/speed*SPIN_RATE)){
				traj.linear.x = nstep / (dist/speed*SPIN_RATE) * dist;
				traj.linear.y = 0;
				traj.linear.z = nstep / (dist/speed*SPIN_RATE) * dist;
				traj.angular.z = 0;
				traj.angular.x = 1;
			}
			else if (nstep <= 2*(dist/speed*SPIN_RATE)){
				traj.linear.x = 2*dist - (nstep / (dist/speed*SPIN_RATE) * dist);
				traj.linear.y = 0;
				traj.linear.z = 2*dist - (nstep / (dist/speed*SPIN_RATE) * dist);
				traj.angular.z = 0;
				traj.angular.x = 1;
			}
			else{
				traj.angular.x = 0;
			}
			traj.angular.y = 1;
			break;
		}

		// Spatial
		case 2:{
			float rr = 3; 
			float hh = 3;
			if (nstep <= (2*PI*rr/speed*SPIN_RATE)) {
				traj.linear.x = rr * sin(nstep / (2*PI*rr/speed*SPIN_RATE) * 2*PI);
				traj.linear.y = rr * (1-cos(nstep / (2*PI*rr/speed*SPIN_RATE) * 2*PI));
				traj.linear.z = nstep / (2*PI*rr/speed*SPIN_RATE) * hh;
				traj.angular.z = 0;
				traj.angular.x = 1;
			}
			else if (nstep <= (2*PI*rr/speed*SPIN_RATE)+(hh/speed*SPIN_RATE)) {
				traj.linear.x = rr * sin(2*PI);
				traj.linear.y = rr * (1-cos(2*PI));
				traj.linear.z = hh - (nstep-2*PI*rr/speed*SPIN_RATE) / (hh/speed*SPIN_RATE) * hh;
				traj.angular.z = 0;
				traj.angular.x = 1;
			}
			else{
				traj.angular.x = 0;
			}
			traj.angular.y = 2;
			break;
		}

		// Sqare
		case 3:{
			float dist = 5;
			if (nstep <= (dist/speed*SPIN_RATE)){
				traj.linear.x = nstep / (dist/speed*SPIN_RATE) * dist;
				traj.linear.y = 0;
				traj.linear.z = 0;
				traj.angular.z = 0;
				traj.angular.x = 1;
			}
			else if (nstep <= 2*(dist/speed*SPIN_RATE)){
				traj.linear.x = (dist/speed*SPIN_RATE) / (dist/speed*SPIN_RATE) * dist;
				traj.linear.y = nstep / (dist/speed*SPIN_RATE) * dist - dist;
				traj.linear.z = 0;
				traj.angular.z = 0;
				traj.angular.x = 1;
			}
			else if (nstep <= 3*(dist/speed*SPIN_RATE)){
				traj.linear.x = 3*dist - (nstep / (dist/speed*SPIN_RATE) * dist);
				traj.linear.y = (dist/speed*SPIN_RATE) / (dist/speed*SPIN_RATE) * dist;
				traj.linear.z = 0;
				traj.angular.z = 0;
				traj.angular.x = 1;
			}
			else if (nstep <= 4*(dist/speed*SPIN_RATE)){
				traj.linear.x = 0;
				traj.linear.y = 4*dist - (nstep / (dist/speed*SPIN_RATE) * dist);
				traj.linear.z = 0;
				traj.angular.z = 0;
				traj.angular.x = 1;
			}
			else{
				traj.angular.x = 0;
			}
			traj.angular.y = 3;
			break;
		}

		// takeoff
		case 8:{
			traj.angular.y = 8;
			trajPub.publish(traj);
			break;
		}

		case 9:{
			traj.angular.y = 9;
			break;
		}
	}
}

