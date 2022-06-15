#ifndef PID_H_INCLUDED
#define PID_H_INCLUDED
typedef struct
{

  float setpoint;       //�趨ֵ
  float kp;     //����ϵ��
  float ki;      //����ϵ��
  float kd;    //΢��ϵ��
  float lasterror;     //ǰһ��ƫ��
  float sec_lasterror;
}PID;

float pid_regultion(PID *pid,float error);
void init_pid(PID*pid,float kp,float ki,float kd);
int speed_to_puls(float speed);
#endif // PID_H_INCLUDED
