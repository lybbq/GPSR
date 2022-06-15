#ifndef __MODEL_H
#define __MODEL_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Motor.h"
#include "tim.h"
#include <stdlib.h>
// #define Odom_HandleTypeDef struct Odom 
#define Min_Wheel_diameter 0.025 // uion = m
#define Mid_Wheel_diameter 0.048 // uion = m
#define Max_Wheel_diameter 0.165 // uion = m
#define Max_Wheel_Length_to_Center 0.18 // uion = m
#define Period_Plus 3000 // a peroid of Min's encode
#define arg 0.0089 // 10ms  max_speed
#define arg1_min 0.0026 * 100// 10ms expand 100. plus multiply this arg = min_real
#define arg_max_to_min 0.29 * 100 // max_speed_pre multiply this arg  = min_speed_pre
#define PI 3.141569
#define Robort_HandleTypeDef struct Robort
#define cosx 0.87
#define sinx 0.5
Robort_HandleTypeDef 
{
	double vx;
	double delta;
	uint8_t sign;
};
extern Robort_HandleTypeDef robot; //define robort
void model_contorl(double vx,double delta,Motor_HandleTypeDef* Motor_1,Motor_HandleTypeDef* Motor_2,Motor_HandleTypeDef* Motor_3);

#ifdef __cplusplus
}
#endif
#endif 