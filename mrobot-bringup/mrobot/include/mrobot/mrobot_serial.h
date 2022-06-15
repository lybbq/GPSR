#ifndef __MROBOT_H
#define __MROBOT_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include "math.h"
#define  Head_Frame 0x30
#define End_Frame 0x40 
#define PI acos(-1)
#define Min_wheel_Pluse 3000
#define Min_wheel_dim 0.025
#define Max_wheel_dim 0.165
#define Max_wheel_r 0.165/2
#define Mid_wheel_dim 0.048
#define Max_wheel_Link 0.18
#define sina sin(PI / 6)
#define cosa cos(PI / 6)
#define dt 0.001 // 1ms
namespace mrobot
{
typedef enum{error_sign = 0xff,right_sign = 0x00, read_speed = 0xfe} receive_sign;
class MRobot
{
public:
    MRobot();
    ~MRobot();
    bool init();
    bool spinOnce(double RobotV, double YawRate);
    bool readTest();
    void odom();
    bool writeSpeed(double RobotV, double YawRate);
    bool readSpeed();
    double cnt;
    double M1_speed;
    double M2_speed;
    double M3_speed;
   
private:
    uint16_t  crc16bitbybit(uint8_t* ptr, uint16_t len);
    unsigned char getCrc8(unsigned char *ptr, unsigned short len);
    void pub_topic(uint8_t Data_Buf[14]);
    bool count_crc_speed(uint8_t *Data_Buf,int len);
    bool write(uint8_t * buf,int len);
   
private:
    ros::Time current_time_, last_time_;

    double x_;
    double y_;
    double th_;

    double max_wheel1_wspeed;
    double max_wheel2_wspeed;
    double max_wheel3_wspeed;

    double vx_;
    double vy_;
    double vth_;
    ros::NodeHandle nh;
    ros::Publisher pub_;
    tf::TransformBroadcaster odom_broadcaster_;
};
    
}

#endif /* MROBOT_H */
