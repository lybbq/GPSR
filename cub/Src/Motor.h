#ifndef __Motor_H
#define __Motor_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "PID.h"
#define Motor_HandleTypeDef struct Mon 
typedef enum {REV = 2,FWD = 1,ST = 0} Mon_status;
typedef enum {motor_one = 1,motor_two = 2,motor_three = 3} Mon_Number;
Motor_HandleTypeDef 
{
	 UART_HandleTypeDef* usart;
	 TIM_HandleTypeDef * htim_encode ;

	 Mon_status ex_Status; //预期的状态
	 Mon_status Status ; //实际状态
	
	 Mon_Number number;
	 uint16_t Encode_Plus;
	
	 double Max_Real_Speed_pre;
	 double Max_Real_Speed ; // m/s  if you need to use this value ,you should  divide 100000
	
	 double min_speed_real;
	 double min_speed_pre;
	 int puls;
	 float set_speed;
	 PID pid;
};
extern Motor_HandleTypeDef Motor_1 ;// extern 
extern Motor_HandleTypeDef Motor_2 ;
extern Motor_HandleTypeDef Motor_3 ;
/* USER CODE BEGIN Includes */
void Motor_control(Motor_HandleTypeDef* Motor,int ratio);
//void Motor_PWM_Con(Motor_HandleTypeDef* Motor,uint16_t Value);
//void Motor_Fwd(Motor_HandleTypeDef* Motor);
//void Motor_Rev(Motor_HandleTypeDef* Motor);
void Motor_Stop(Motor_HandleTypeDef* Motor);

#ifdef __cplusplus
}
#endif
#endif /*__ Motor_H */