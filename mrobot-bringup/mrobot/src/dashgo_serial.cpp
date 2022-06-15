#include "mrobot/mrobot_serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>   //输入输出库
namespace mrobot{

    boost::asio::io_service iosev;
    boost::asio::serial_port sp(iosev, "/dev/ttyUSB0"); 
    MRobot::MRobot():
        x_(0.0), y_(0.0), th_(0.0),
        vx_(0.0), vy_(0.0), vth_(0.0)
    {
    }
    MRobot::~MRobot()
    {

    }

    bool MRobot::init() 
    {
        // 串口连接
        sp.set_option(boost::asio::serial_port::baud_rate(115200));
        		sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        	sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        sp.set_option(boost::asio::serial_port::character_size(8));
        cnt = 0;
        ros::Time::init();
        current_time_ = ros::Time::now();
        last_time_ = ros::Time::now();
        
        //定义发布消息的名称
        //pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);		
        
        return true;
    }
  bool MRobot::write(uint8_t*  buf,int len) // set size  = 9 send
  {
        uint8_t buf_copy[9] ;
        for (int i = 0;i < len ;i++) buf_copy[i] = buf[i];
        boost::asio::write(sp, boost::asio::buffer(buf_copy)); // write data;
        double current_time_ = ros::Time::now().toSec();
        while(!readTest())
        { 
            // std::cout << " Slave  receive data";
            boost::asio::write(sp, boost::asio::buffer(buf_copy));
            double last_time_ = ros::Time::now().toSec();
            if(last_time_  - current_time_ >= 3) 
            {
                return false;
            }

        }
        //std::cout << " Slave  receive data";
        return true;
  }
  bool MRobot::readSpeed()
  {
       uint8_t buf[9] = {Head_Frame,read_speed,0,0,0,0,End_Frame,0,End_Frame};
        uint16_t CRC =crc16bitbybit(buf,7);
        buf[6] = (uint8_t )((CRC &0XFF00) >> 8);
        buf[7] = (uint8_t )(CRC &0X00FF);
        return write(buf,9);
  }

    bool MRobot::writeSpeed(double RobotV, double YawRate)
    {
        uint8_t RobotV_ = (uint8_t)abs((RobotV * 10));
        uint8_t YawRate_= (uint8_t)abs((YawRate*10));
        uint8_t RobotV_Pol = (RobotV == 0) ? 0 : ((RobotV > 0)? 1:2);
        uint8_t YawRate_Pol= (YawRate == 0) ? 0 : ((YawRate> 0)? 1:2);
        uint8_t buf[9] = {Head_Frame,0x50,RobotV_Pol,RobotV_,YawRate_Pol,YawRate_,End_Frame,0,End_Frame};

        uint16_t CRC =crc16bitbybit(buf,7);
        buf[6] = (uint8_t )((CRC &0XFF00) >> 8);
        buf[7] = (uint8_t )(CRC &0X00FF);
       /* for (int i =0 ; i< 9;i++)
        {
                std::cout << std:: hex << (unsigned int)buf[i] << ",";
        }*/
        // cnt crc
        /*boost::asio::write(sp, boost::asio::buffer(buf)); // write data;

        // ensure slave  receive data
      double current_time_ = ros::Time::now().toSec();
        while(!readTest())
        { 
            boost::asio::write(sp, boost::asio::buffer(buf));
            double last_time_ = ros::Time::now().toSec();
            if(last_time_  - current_time_ >= 3) 
            {
                return false;
            }

        }*/
        //std::cout << " Slave  receive data";
        return write(buf,9);
    }
    bool MRobot::readTest()
    {
            uint8_t buf[8]; //serial store buf 

            // 读取串口数据
            boost::asio::read(sp, boost::asio::buffer(buf));
            /*for(int i = 0;i<8;i++)
            {
                std::cout << std::hex << (unsigned int)buf[i] << ",";
            }*/
            if(buf[0] != Head_Frame || buf[7] != End_Frame || buf[1] == error_sign) 
                 return false  ;
            if(buf[1] == read_speed)
            {
                M1_speed = buf[2] / 10;
                M2_speed = buf[3] / 10;
                M3_speed = buf[4] / 10;
            }
            return true;

    }
    uint16_t  MRobot::crc16bitbybit(uint8_t* ptr, uint16_t len)
    {
        uint8_t i;
        uint16_t  crc = 0xffff;
        const uint16_t polynom = 0xA001;
        if (len == 0) {
            len = 1;
        }
        while (len--) {
            crc ^= *ptr;
            for (i = 0; i < 8; i++)
            {
                if (crc & 1) {
                    crc >>= 1;
                    crc ^= polynom;
                }
                else {
                    crc >>= 1;
                }
            }
            ptr++;
        }
        return(crc);

    }
}
