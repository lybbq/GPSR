#include "Motor.h"
#include "tim.h"
#include "usart.h"
#include "send.h"
Motor_HandleTypeDef Motor_1={&huart2,&htim2,ST,ST,motor_one,0,0,0,0,0,0,0}; // Motor 1
Motor_HandleTypeDef Motor_2={&huart3,&htim4,ST,ST,motor_two,0,0,0,0,0,0,0}; // Motor 2
Motor_HandleTypeDef Motor_3={&huart6,&htim5,ST,ST,motor_three,0,0,0,0,0,0,0}; // Motor 3
// Motor_HandleTypeDef Motor_2={TIM_CHANNEL_2,&htim3,GPIO_PIN_5,GPIO_PIN_2,GPIOA,GPIOA,ST,two}; // Motor 1
// fwd
/***************************************************/
/**             **/
/***************************************************/
void Motor_control(Motor_HandleTypeDef* Motor,int ratio)
{
	// ratio range is -1000 to 1000
	Usart_motor_ratio(Motor->usart,Motor->number,ratio);

	
}

void Motor_Stop(Motor_HandleTypeDef* Motor)
{
	Usart_motor_ratio(Motor->usart,Motor->number,0);
	 //Motor.Status = ST;
}