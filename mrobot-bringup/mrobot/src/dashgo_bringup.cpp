#include "mrobot/mrobot_serial.h"
#include <iostream>
double RobotV_ = 0;
double YawRate_ = 0;
bool callback_sign =false;
void cmdCallback(const geometry_msgs::Twist& msg)
{
	RobotV_ = msg.linear.x ;
	YawRate_ = msg.angular.z;
    callback_sign = true ;
    //std::cout << "11";
}
int main(int argc, char** argv)
{
    //初始化ROS节点
	ros::init(argc, argv, "dashgo_bringup");									
    ros::NodeHandle nh;
    
    //初始化MRobot
	mrobot::MRobot robot;
    if(!robot.init())
        ROS_ERROR("MRobot initialized failed.");
	ROS_INFO("MRobot initialized successful.");
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 50, cmdCallback); //  Subscriber  cmd_vel
    ros::Rate loop_rate(10);
    bool stop_sign = false;
    while (ros::ok()) 
    {
          // ROS_INFO("MRobot initialized successful.");
          //std::cout << "22";
          ros::spinOnce();
          if(callback_sign) 
            {
                  if(!robot.writeSpeed(RobotV_,YawRate_ ))
                    ROS_ERROR("Data_Error or  Slave  no run");
                 callback_sign = false;
                 stop_sign = false;
            }else if(!stop_sign)      
                {stop_sign = robot.writeSpeed(0,0);
		ROS_INFO("stop");}

            //if(robot.readSpeed())
                //ROS_INFO("M1_Speed = %f,M2_Speed = %f,M3_Speed = %f",robot.M1_speed,robot.M2_speed,robot.M3_speed);
            //else
                 //ROS_ERROR("Data_Error or  Slave  no run");
		 loop_rate.sleep();
          //ROS_INFO("MRobot initialized successful.");
	}
	return 0;
}
