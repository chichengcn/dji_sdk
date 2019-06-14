#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "xbee.h"

const char* tag = "#02";
struct timeval prtime;

serial::Serial ser;

void xbee_write_callback(const std_msgs::String::ConstPtr& msg){
    //ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "xbee");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("xbee_write", 1000, xbee_write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("xbee_read", 1000);


    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

/*    int _cnt = 0;*/
    ros::Rate loop_rate(1000);
    while(ros::ok()){

        ros::spinOnce();

/*        _cnt++;
        if (_cnt == 5){
            send_timeStamp();
            _cnt = 0;
        }*/

        if(ser.available()){
            //ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            //ROS_INFO_STREAM("Read: " << result.data);
            ser.write(result.data);
            //get_data(result);
            //read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}


static uint8_t chksum(char *dataptr)
{
  uint8_t i, chs;

  for(chs=dataptr[1],i=2;dataptr[i]!='*';i++)
  {
    chs ^= dataptr[i];
  }

  return chs;
}

void send_timeStamp(void)
{
    time_t _time;
    struct tm* lctime;
    char _msg[100];

    time(&_time);
    lctime = localtime(&_time);

    sprintf(_msg, "%s$TIME,%d-%d-%d,%d:%d:%d*", tag, lctime->tm_year+1900, lctime->tm_mon, lctime->tm_mday, lctime->tm_hour, lctime->tm_min, lctime->tm_sec);
    uint8_t chs = chksum(_msg);
    char str_chs[10];
    sprintf(str_chs, "%x\r\n", chs);
    strcat(_msg, str_chs);
    
    ser.write(_msg);
    gettimeofday(&prtime, NULL);
}

void get_data(std_msgs::String msg)
{
    int idx;
    int rc_tag;
    struct timeval prtime_now;
    int dtime;

    if (idx = msg.data.find("$TIME") != string::npos){
        gettimeofday(&prtime_now, NULL);
        dtime = (prtime_now.tv_usec - prtime.tv_usec) / 1000;
        ROS_INFO("dtime: %dms", dtime);
    }
}


