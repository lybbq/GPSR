#include "encode.h"
#include "send.h"
#include "usart.h"
#include "model.h"
void Encode_Motor(Motor_HandleTypeDef* Motor,uint32_t code_init_value)
{
	uint32_t temp_encode_value;
	int32_t encode_value;
	Mon_Number mon_number = Motor->number;
	temp_encode_value = (uint32_t)(__HAL_TIM_GET_COUNTER(Motor->htim_encode));  // 
	encode_value = temp_encode_value - code_init_value; //
	switch(mon_number)
	{
		case motor_one:
		{
			Motor->Status = (encode_value == 0) ? ST:((encode_value > 0 )? FWD:REV);
			double speed = (double)(abs(encode_value))/(Period_Plus);
			//if(temp_encode_value != code_init_value)
				//Usart_Unin32 (&huart1,(uint32_t)(abs(encode_value)*arg));
			break;
		}
		case motor_two:
		{
			// Usart_Unin32 (&huart1,temp_encode_value);
		 Motor->Status = (encode_value == 0) ? ST:((encode_value > 0 )? FWD:REV);
		 if(temp_encode_value != code_init_value)
		 {
				Usart_Unin32 (&huart1,(uint32_t)(abs(encode_value)*arg1_min));
			  // Usart_Unin32(&huart1,(uint32_t)(abs(Motor->min_speed_pre)));
			  //Usart_Unin32 (&huart1,(uint32_t)(Motor->Status));
		 }
		
			break;
		}
		case motor_three:
		{
			//if(temp_encode_value != code_init_value)
				//Usart_Unin32 (&huart1,(uint32_t)(abs(encode_value)*arg));
			Motor->Status = (encode_value == 0) ? ST:((encode_value > 0 )? FWD:REV);
			break;
		}
	}
	// encode_value = abs(encode_value); 
	Motor->Max_Real_Speed = (double)(encode_value)*arg;        //max_wheel_speed   
  Motor->min_speed_real = encode_value * arg1_min;
	
	// Usart_Unin32(&huart1,(uint32_t)(abs(Motor->min_speed_real)));
	//Usart_Unin32(&huart1,(uint32_t)(abs(Motor->set_speed)));
	// Usart_Unin32(&huart1,(uint32_t)(abs(Motor->puls)));
	
	 __HAL_TIM_SET_COUNTER(Motor->htim_encode,code_init_value); 
}