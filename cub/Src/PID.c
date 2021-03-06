#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "PID.h"

float pid_regultion(PID *pid,float error) /*vPID是指针指向结构体，robort实时调控数值*/
{
float thiserror;//
float perror,ierror,derror;
float lasterror;//
float result;
thiserror = error;//
perror = thiserror - pid->lasterror;//
ierror = thiserror;//
derror = thiserror + pid->sec_lasterror - 2 * pid->lasterror; //

// result = pid->kp * perror + pid->ki*ierror + pid->kd * derror;
result = pid->kp * perror + pid->ki * ierror + pid->kd * derror;

pid->sec_lasterror = pid->lasterror;
pid->lasterror = thiserror;//
return  result;
}
void init_pid(PID*pid,float kp,float ki,float kd)
{
pid->kp = kp;     //比例系数
pid->ki = ki;      //积分系数
pid->kd = kd;    //微分系?
pid->lasterror = 0;     //前一拍偏差
pid->sec_lasterror = 0;
 //输出值
}
int speed_to_puls(float speed)
{
	int plus =0;
	float y_set = abs(speed);
	float w1 = 0.436,b1 = -7.33;
	float w2 = 0.168,b2 = 50.998;
	float w3 = 0.842,b3 = 102.89;
	
	int polarity = (speed >= 0)? 1:-1;
	
	if(y_set <= 200) plus = (y_set -b1)/ w1;
	else if(y_set >200 && y_set <=400) plus = (y_set -b2)/ w2;
	else plus = (y_set - b3)/ w3;
	
	return plus*polarity ;
}
/*int main()
{
float n=0,i=0,k=20;
PID *pid;
pid=(PID*)malloc(sizeof(PID));
INIT_PID(pid,0x200);
while(i<=100)
{
    PID_regultion(pid,0x199);//PID函数
    printf("%d\n",pid->result);
    i++;
}

}*/
