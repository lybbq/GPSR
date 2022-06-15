#include "model.h"
Robort_HandleTypeDef robot = {0,0,0};//robort model

void model_contorl(double vx,double delta,Motor_HandleTypeDef* Motor_1,Motor_HandleTypeDef* Motor_2,Motor_HandleTypeDef* Motor_3)
{
	// model vy = 0 so  simple model 
	// model vy = 0 so  simple model 
	// speed unio ~m/s
	double M2 = 0 - vx*0.87 + delta * Max_Wheel_Length_to_Center; // vol 
	double M1 =  vx*0.87 + delta * Max_Wheel_Length_to_Center; // vol 
	double M3 = delta * Max_Wheel_Length_to_Center; // vol 
	
	Motor_1->Max_Real_Speed_pre = M1;
	Motor_2->Max_Real_Speed_pre = M2;
	Motor_3->Max_Real_Speed_pre = M3;
	
	Motor_1->min_speed_pre  = M1 * arg_max_to_min;
	Motor_2->min_speed_pre  = M2 * arg_max_to_min;
	Motor_3->min_speed_pre  = M3 * arg_max_to_min;
	
	
	
	Mon_status M1_Pre = (M1 == 0)? ST:((M1 > 0)? FWD : REV);
	Mon_status M2_Pre = (M2 == 0)? ST:((M2 > 0)? FWD : REV);
	Mon_status M3_Pre = (M3 == 0)? ST:((M3 > 0)? FWD : REV);
	
	// REV  Must stop
	if(M1_Pre != Motor_1->Status || M2_Pre != Motor_2->Status||M3_Pre != Motor_3->Status)
	{
		Motor_Stop(Motor_1);
		Motor_Stop(Motor_2);
		Motor_Stop(Motor_3);
		
		Motor_1->pid.lasterror = 0,Motor_2->pid.lasterror = 0,Motor_3->pid.lasterror = 0;
		Motor_1->pid.sec_lasterror = 0,Motor_1->pid.sec_lasterror = 0,Motor_3->pid.sec_lasterror = 0;
		
		while(Motor_1->Status != ST && Motor_2->Status != ST && Motor_3->Status != ST);
	}
	
	//if(Motor_1->min_speed_pre == Motor_1->set_speed);
	//if(Motor_2->min_speed_pre == Motor_2->set_speed);
	//if(Motor_3->min_speed_pre == Motor_3->set_speed);
	// find coor speed to ratio
	//write the speed.
  //Usart_Unin32 (&huart1,temp_encode_value);
  
}