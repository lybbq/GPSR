#ifndef __Send_H
#define __Send_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Motor.h"
#include "encode.h"
#include "stdlib.h"
#include "usart.h"
#include "model.h"
	 
void Usart_Unin32 (UART_HandleTypeDef *huart,uint32_t value);
typedef enum {Speed_Tpye = 0x40,Odom_Tpye = 0x50} Data_Type;
typedef enum {Positive  = 0x01,Minus = 0x00} Symbol_Pority;
typedef enum {error_sign  = 0xff,right_sign = 0x00,motor_stop = 0xf1,read_speed_sign = 0xfe,write_speed = 0x50} Encode;
void Usart_motor_ratio(UART_HandleTypeDef *huart,int number,int ratio);
void Hanlde_receive(uint8_t* buf,uint8_t len,Robort_HandleTypeDef* robot,Motor_HandleTypeDef* M1,Motor_HandleTypeDef* M2, Motor_HandleTypeDef* M3);
void Usart_Data_MotorsPlus(UART_HandleTypeDef *huart,Motor_HandleTypeDef Motor1,Motor_HandleTypeDef Motor2,Motor_HandleTypeDef Motor3);
#ifdef __cplusplus
}
#endif
#endif /*__ Motor_H */