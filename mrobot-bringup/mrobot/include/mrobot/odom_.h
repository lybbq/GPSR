#ifndef __ODOM_H
#define __ODOM_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include "mrobot/Serial_msg.h" 
#include "math.h"
#define PI acos(-1)
#define Min_wheel_Pluse 2000
#define Min_wheel_dim 0.025
#define Max_wheel_dim 0.165
#define Mid_wheel_dim 0.048
#define Max_wheel_Link 0.18
#define sina sin(PI / 6)
#define cosa cos(PI / 6)

void odom(ros::Time current_time_ ,double & x,double &y,double &th,double &max_wheel1_speed ,double & masx_wheel2_speed,double & masx_wheel3_speed);
bool data_crc(const mrobot::Serial_msg & data,double & max_wheel1_speed,double & max_wheel2_speed,double & max_wheel3_speed);
    


#endif /* MROBOT_H */
