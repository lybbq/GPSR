#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED
typedef struct
{

  float setpoint;       //设定值
  float kp;     //比例系数
  float ki;      //积分系数
  float kd;    //微分系数
  float lasterror;     //前一拍偏差
  float sec_lasterror;
}PID;

float pid_regultion(PID *pid,float error);
void init_pid(PID*pid,float kp,float ki,float kd);
int speed_to_puls(float speed);
#endif // PID_H_INCLUDED
