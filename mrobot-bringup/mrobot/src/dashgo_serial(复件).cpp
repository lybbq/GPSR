#include "mrobot/mrobot_serial.h"
#include <stdio.h>
 #include <stdlib.h>
#include <vector>
  #include <iostream>   //输入输出库
namespace mrobot{

    boost::array<double, 36> odom_pose_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3, 1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
    boost::array<double, 36> odom_twist_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3, 1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0, 
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
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
        pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);		
        
        return true;
    }
    bool MRobot::count_crc_speed(uint8_t* Data_Buf,int len)
{
            uint8_t  Data_Buf_Length = len;
            uint8_t  Parity_Buf[Data_Buf_Length-2];  // use CRC
            uint8_t  Parity_Buf_Length = sizeof(Parity_Buf);
            uint8_t Crc_Higt_Data = Data_Buf[Data_Buf_Length-3];
            uint8_t Crc_Low_Data = Data_Buf[Data_Buf_Length-3];
            uint8_t Head_Frame = 0x30; 
            uint8_t End_Frame = 0x40;
            // end_sign = false;
            /*                                                         ensure start frame and end frame                                                                     */
            if (Data_Buf[0] != Head_Frame && Data_Buf[Data_Buf_Length-1] != End_Frame) return false;
            /*                                                         Parity_Buf to cont Crc                                                                     */
            memcpy(Parity_Buf,Data_Buf,Data_Buf_Length-3);
            Parity_Buf[Parity_Buf_Length-1] = Data_Buf[Data_Buf_Length-1]; // end frame
            /*                                                         Conut Crc                                                                     */
            uint16_t  Crc = crc16bitbybit(Parity_Buf,Parity_Buf_Length);
            uint8_t Count_Crc_High = (uint8_t) ((Crc & 0xff00) >> 8);
            uint8_t Count_Crc_Low = (uint8_t) (Crc & 0x00ff) ;
            if (Count_Crc_High != Crc_Higt_Data &&  Count_Crc_Low != Crc_Low_Data ) return false;
            /*                                                         Conut Speed                                                                     */
            uint8_t Fun_Code = Parity_Buf[1];
            if ( Fun_Code  == 0x50)
            {
                uint8_t Polarity_Motor1 = Parity_Buf[2];
                uint8_t Polarity_Motor2 = Parity_Buf[5];
                uint8_t Polarity_Motor3 = Parity_Buf[8];


                double Motor1_Plus = ((Parity_Buf[3] << 8) | Parity_Buf[4]) ;
                double Motor2_Plus = (Parity_Buf[6] << 8) | Parity_Buf[7] ;
                double Motor3_Plus = (Parity_Buf[9] << 8) | Parity_Buf[10] ;

                /*   0  stop  1 Fwd  2 REV*/
                Motor1_Plus =(Polarity_Motor1 > 0) ? (Polarity_Motor1 == 1 ? Motor1_Plus : 0- Motor1_Plus) : 0;
                Motor2_Plus =(Polarity_Motor2 > 0) ? (Polarity_Motor2 == 1 ? Motor2_Plus : 0- Motor2_Plus) : 0;
                Motor3_Plus =(Polarity_Motor3 > 0) ? (Polarity_Motor3 == 1 ? Motor3_Plus : 0- Motor3_Plus) : 0;

                /*std::cout << " Speed 1 = " << Motor1_Plus << std::endl;
                std::cout << " Speed 2 = " << Motor2_Plus << std::endl;
                std::cout << " Speed 3 =" << Motor3_Plus << std::endl;*/

                max_wheel1_wspeed = ((Motor1_Plus / Min_wheel_Pluse) * (PI *Min_wheel_dim) / dt)/ (Mid_wheel_dim/2);
                max_wheel2_wspeed = ((Motor2_Plus / Min_wheel_Pluse) * (PI *Min_wheel_dim )/dt) / (Mid_wheel_dim/2);
                max_wheel3_wspeed = ((Motor3_Plus / Min_wheel_Pluse) * (PI *Min_wheel_dim)/dt) / (Mid_wheel_dim/2);
                
                //cnt  += max_wheel2_wspeed * dt  * (Max_wheel_dim/2);
                //std::cout <<cnt << std ::endl ;
                /*if(max_wheel1_wspeed || max_wheel2_wspeed || max_wheel3_wspeed)
                        {
                            std::cout << " Speed 1 = " << max_wheel1_wspeed << ' ';
                            std::cout << " Speed 2 = " << max_wheel2_wspeed << ' ';
                            std::cout << " Speed 3 =" << max_wheel3_wspeed << std::endl;
                        }*/
            }
        return true;
}
    bool MRobot::readTest()
    {
            uint8_t buf[14]; //serial store buf 
            static uint8_t Data_Buf[14]; // 存放正确数据
            uint8_t Head_Frame = 0x30; 
            uint8_t End_Frame = 0x40;
            bool  crc_sign = false ; // 拼接的正确帧 sign
            static  uint8_t offset ;

            // 读取串口数据
            boost::asio::read(sp, boost::asio::buffer(buf));
        uint8_t  frame_size = sizeof(buf);
            for (int i = 0;i < sizeof(buf);i++)
            {
                    if(buf[i] == Head_Frame)
                    {
                            Data_Buf[0] = Head_Frame;
                            memcpy( Data_Buf,&buf[i],sizeof(buf)-i);
                            offset = sizeof(buf)-i;
                            if(offset == 14) 
                                {
                                    crc_sign = count_crc_speed(Data_Buf,frame_size);
                                    break;
                                }
                    }
                    else if (buf[i] == End_Frame)
                    {
                            memcpy( &Data_Buf[offset],buf,i+1);
                            crc_sign = count_crc_speed(Data_Buf,frame_size);
                    }
        }
        return crc_sign;
    }
    void MRobot::odom()
    {
        /*  a = 30 L =180 R = 165*/
        /*  count Vx Vy deta*/
        /*bool sign = readTest();
        if(! sign)
        {
            std::cout << "Data error ";
            return;
        }*/
        double  cosx = cos(th_); 
        double  sinx = sin(th_);

        vx_ = ((2*max_wheel3_wspeed - max_wheel2_wspeed - max_wheel1_wspeed)*Max_wheel_r) / (2*sina + 2);
        vth_ = (max_wheel3_wspeed *Max_wheel_r  - vx_) / Max_wheel_Link ;
        vy_ = ( vth_ *Max_wheel_Link - vx_*sina - max_wheel2_wspeed * Max_wheel_r  ) / cosa;

      
       // vx_ = 0.11 * cosx * max_wheel3_wspeed - max_wheel2_wspeed * (0.55 * cosx - 0.55 * cosa * sinx) - max_wheel1_wspeed * (0.55 * cosx + 0.55 * cosa * sinx);
        //vy_ = 0.11* sinx * max_wheel3_wspeed - max_wheel2_wspeed * (0.55 * sinx + 0.55 * cosa *cosx) - max_wheel1_wspeed * (0.55 * sinx - 0.55 * cosa *cosx);
        //vth_ = (11 * max_wheel1_wspeed) / 36 + (11 * max_wheel1_wspeed)/36+(11 * max_wheel3_wspeed)/36;
        double dx_ = (cosx * vx_ - sinx * vy_) * dt;
        double dy_ = (sinx * vx_ + cosx *vy_) * dt;
        double dth_ = vth_ * dt;

        x_ += dx_ ;
        y_ += dy_ ;
        th_ += dth_ ;
        
         std::cout << " x = " << x_ << ' ';
        std::cout << " y = " << y_<< ' ';
        std::cout << " w =" << th_ << std::endl;
    // current_time_ = ros::Time::now();
    // 发布TF
    current_time_ = ros::Time::now();
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time_;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_footprint";
    // std:: cout << "in "<<std::endl;
    geometry_msgs::Quaternion odom_quat;
    odom_quat = tf::createQuaternionMsgFromYaw(th_);
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    
    odom_broadcaster_.sendTransform(odom_trans);

    // 发布里程计消息
    nav_msgs::Odometry msgl;
    msgl.header.stamp = current_time_;
    msgl.header.frame_id = "odom";

    msgl.pose.pose.position.x = x_;
    msgl.pose.pose.position.y = y_;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_footprint"; //base_footprint
    msgl.twist.twist.linear.x = vx_;
    msgl.twist.twist.linear.y = vy_;
    msgl.twist.twist.angular.z = vth_;
    msgl.twist.covariance = odom_twist_covariance;
  
    pub_.publish(msgl);
    }
    uint16_t  MRobot::crc16bitbybit(uint8_t* ptr, uint16_t len)
    {
        uint8_t i;
        uint16_t crc = 0xffff;
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
