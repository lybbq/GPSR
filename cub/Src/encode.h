#ifndef __ENCODE_H
#define __ENCODE_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Motor.h"
#include "tim.h"
#include <stdlib.h>
// #define Odom_HandleTypeDef struct Odom 
// #define Min_Wheel_diameter 0.025 // uion = m
// #define Mid_Wheel_diameter 0.048 // uion = m
// #define Max_Wheel_diameter 0.165 // uion = m
// #define Max_Wheel_Length_to_Center 0.18 // uion = m
// #define Period_Plus 1500 // a peroid of Min's encode
// #define PI 3.141569
void Encode_Motor(Motor_HandleTypeDef* Motor,uint32_t code_init_value);
#ifdef __cplusplus
}
#endif
#endif /*__ Motor_H */