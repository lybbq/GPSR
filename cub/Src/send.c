#include "send.h"
//char Table[10]={'0','1','2','3','4','5','6','7','8','9'};
#define Frame_Head 0x30  //头帧
#define Frame_End 0x40	//尾帧
uint16_t crc16bitbybit(uint8_t* ptr, uint16_t len)
{
	uint8_t i;
	uint16_t crc = 0xffff;
	const uint16_t polynom = 0xA001;
	if (len == 0) {
		len = 1;
	}
	while (len--) {
		crc ^= *ptr;
		for (i = 0; i < 8; i++)
		{
			if (crc & 1) {
				crc >>= 1;
				crc ^= polynom;
			}
			else {
				crc >>= 1;
			}
		}
		ptr++;
	}
	return(crc);

}
void Tran_Un32_To_Char(unsigned int temp,uint8_t * store)
{
	store[0]=temp%10000000000/1000000000; //????
	store[1]=temp%1000000000/100000000; //???
	store[2]=temp%100000000/10000000; //???
	store[3]=temp%10000000/1000000; //???
	store[4]=temp%1000000/100000; //???
	store[5]=temp%100000/10000; //??
	store[6]=temp%10000/1000; //??
	store[7]=temp%1000/100; //??
	store[8]=temp%100/10; //??
	store[9]=temp %10 ;//??
	store[10]='\n'+0x30;
}

uint8_t Tran_Un32_To_Char2(unsigned int temp,uint8_t * Tx)
{
	uint8_t store[10];
	int i = 0;
	int index = 0;
	uint8_t sign = 0;
	uint8_t size ;
	// if(!temp) return 0;
	store[0]=temp%10000000000/1000000000; //????
	store[1]=temp%1000000000/100000000; //???
	store[2]=temp%100000000/10000000; //???
	store[3]=temp%10000000/1000000; //???
	store[4]=temp%1000000/100000; //???
	store[5]=temp%100000/10000; //??
	store[6]=temp%10000/1000; //??
	store[7]=temp%1000/100; //??
	store[8]=temp%100/10; //??
	store[9]=temp %10 ;//??
	size = sizeof(store);
	for (i = 0; i < size; i++)
	{
		if (!sign && !store[i]) // 第一位不为0的数开始算
			continue;
		else 
		{
			sign = 1;
			Tx[index++]=store[i]+0x30;
		}
	}
	return index;
}
void Usart_motor_ratio(UART_HandleTypeDef *huart,int number,int ratio)
{
	int adr = number;
	int Crc;
	int Crc_High;
	int Crc_Low;
	uint8_t high_ratio = (uint8_t)((ratio & 0xff00) >> 8);
	uint8_t low_ratio = (uint8_t)(ratio & 0x00ff);
	
	uint8_t Tx_Buff[8] = {adr,0x06,0x00,0x40,high_ratio,low_ratio,0,0};
  Crc = crc16bitbybit(Tx_Buff,6);

  Crc_High = (uint8_t)((Crc&0xff00) >> 8);
	Crc_Low = (uint8_t)(Crc&0x00ff);
	/*                       Assign CRC To TX Buff               */
	Tx_Buff[7] = Crc_High;
	Tx_Buff[6]	= Crc_Low;
  HAL_UART_Transmit(huart,Tx_Buff,8,2000);
	//HAL_UART_Transmit(&huart1,Tx_Buff,8,2000);
}
void Usart_Unin32 (UART_HandleTypeDef *huart,unsigned int value)
{
	uint8_t  store[10]; //转变成char 数组
	uint8_t  Tx[10]; //要发送的数组
	int i = 0;
	int index = 0;
	uint8_t sign = 0;
	if (!value)
	{
		uint8_t temp_value = 0+0x30 ;
		uint8_t temp_sign = '\n'+0x30;
		HAL_UART_Transmit(huart,&temp_value,1,2000);
		HAL_UART_Transmit(huart,&temp_sign,1,2000);
	}
	else
	{
		Tran_Un32_To_Char(value, store); // 提取每一位的整数
		uint8_t size = sizeof(store);
		for (i = 0; i < size; i++)
		{
			if (!sign && !store[i]) // 第一位不为0的数开始算
				continue;
			else 
			{
				sign = 1;
				Tx[index++]=store[i]+0x30;
			}
		}
		for (i = 0; i < index; i++)
		{
			//if(Tx[i]!= '\n')
			HAL_UART_Transmit(huart,&Tx[i],1,2000);
		}
		HAL_UART_Transmit(huart,&store[10],1,2000);
	}
}
void Usart_Data_MotorsPlus(UART_HandleTypeDef *huart,Motor_HandleTypeDef Motor1,Motor_HandleTypeDef Motor2,Motor_HandleTypeDef Motor3)
{
	/* Frame : Head DateTpye x_pority x_High8 x_Low8 y_pority y_High8 y_Low8 delta_pority delta_High8 delta_Low8 CRC_High8 CRC_Low8 Frame_END*/
	// uint8_t Tx_Crc_Buff[12] = {Frame_Head,0,0,0,0,0,0,0,0,0,0,Frame_End};
	uint8_t Tx_Buff[14] = {Frame_Head,0,0,0,0,0,0,0,0,0,0,Frame_End,0,Frame_End};
	uint16_t Crc ;
	uint8_t Crc_High ;
	uint8_t Crc_Low ;
	/*                       Data Hanlde                          */
	uint8_t Motor1_High = (uint8_t)((Motor1.Encode_Plus & 0xff00) >> 8);
	uint8_t Motor1_Low = (uint8_t)(Motor1.Encode_Plus & 0x00ff);
	uint8_t Motor2_High = (uint8_t)((Motor2.Encode_Plus & 0xff00) >> 8);
	uint8_t Motor2_Low = (uint8_t)(Motor2.Encode_Plus & 0x00ff);
	uint8_t Motor3_High = (uint8_t)((Motor3.Encode_Plus & 0xff00) >> 8);
	uint8_t Motor3_Low = (uint8_t)(Motor3.Encode_Plus & 0x00ff);
	/*                       Assign Data To TX Buff               */
	Tx_Buff[1] = Odom_Tpye;
	Tx_Buff[2] = Motor1.Status;
	Tx_Buff[3] = Motor1_High;
	Tx_Buff[4] = Motor1_Low;
	Tx_Buff[5] = Motor2.Status;
	Tx_Buff[6] = Motor2_High;
	Tx_Buff[7] = Motor2_Low;
	Tx_Buff[8] = Motor3.Status;
	Tx_Buff[9] = Motor3_High;
	Tx_Buff[10] = Motor3_Low;
	/*                       Count CRC                           */
	Crc = crc16bitbybit(Tx_Buff,12);
	Crc_High = (uint8_t)((Crc&0xff00) >> 8);
	Crc_Low = (uint8_t)(Crc&0x00ff);
	/*                       Assign CRC To TX Buff               */
	Tx_Buff[11] = Crc_High;
	Tx_Buff[12]	= Crc_Low;
	/*                       Send Data                           */
	HAL_UART_Transmit(huart,Tx_Buff,14,2000);
	
} 
void send_code(UART_HandleTypeDef *huart,Encode code,Motor_HandleTypeDef M1,Motor_HandleTypeDef M2, Motor_HandleTypeDef M3)
{
	uint8_t Tx_Buff[8] = {Frame_Head,0xf1,0,0,0,Frame_End,0,Frame_End};
	uint16_t Crc ;
	uint8_t Crc_High ;
	uint8_t Crc_Low ;
	switch(code)
	{
		case error_sign:{Tx_Buff[1] = error_sign;break;}
		case right_sign:{Tx_Buff[1] = right_sign;break;}
		case motor_stop:{Tx_Buff[1] = motor_stop;break;}
		case read_speed_sign:
		{
			Tx_Buff[1] = read_speed_sign; 
			Tx_Buff[2] = M1.Max_Real_Speed;
			Tx_Buff[3] = M2.Max_Real_Speed;
			Tx_Buff[4] = M3.Max_Real_Speed;
			break;
		}
	}
  Crc = crc16bitbybit(Tx_Buff,5);
  Crc_High = (uint8_t)((Crc&0xff00) >> 8);
	Crc_Low = (uint8_t)(Crc&0x00ff);

  Tx_Buff[5] = Crc_High;
  Tx_Buff[6] = Crc_Low;

  HAL_UART_Transmit(huart,Tx_Buff,8,2000);
 
}
void Hanlde_receive(uint8_t* buf,uint8_t len,Robort_HandleTypeDef* robot,Motor_HandleTypeDef* M1,Motor_HandleTypeDef* M2, Motor_HandleTypeDef* M3)
{
	// buf[0]= head buf[1]=mode buf[2]=polarity_speed buf[3]=speed buf[4]=polarity_thea buf[5]=thea_value buf[6]=rcc_high buf[7]=rcc_low
	uint8_t Crc_High = buf[len-3];
	uint8_t Crc_Low = buf[len-2];
	uint8_t Crc_High_c = 0;
	uint8_t Crc_Low_c = 0;
	uint16_t Crc ;
	uint8_t mode;
	buf[len-3] = Frame_End; // 数据应是倒数第三位结尾
	if(!len || buf[0]!=Frame_Head ||buf[len-1]!=Frame_End)
	{
		// Error_data(&huart1);
		send_code(&huart1,error_sign,*M1,*M2,*M3);
		return;
	}
	
	// count CRC 
	Crc = crc16bitbybit(buf,len-3);
	Crc_High_c = (uint8_t)((Crc&0xff00) >> 8);
	Crc_Low_c = (uint8_t)(Crc&0x00ff);
	if(Crc_High_c != Crc_High || Crc_Low != Crc_Low_c)
	{
		// Error_data(&huart1);
		send_code(&huart1,error_sign,*M1,*M2,*M3);
		return;		
	}
	// Right_data(&huart1); //tell master , data is right
	mode = buf[1];
	switch(mode) // deal data
	{
		case write_speed://{head fun speed_polorty vx speed_polorty delta}
		{
			send_code(&huart1,right_sign,*M1,*M2,*M3);  //告诉主机 已经写入速度
			
			double vx = (double)(buf[3])/10; // 上位机发送的速度值一般为小数 单位m/s
			double delta = (double)(buf[5])/10;
			// buf[2]== 0 ,is zero, == 1 is position ,==2 is no pois
			vx = (buf[2] == 0)? 0:((buf[2] == 1)? vx : (0-vx));
			delta = (buf[4] == 0)? 0:((buf[4] == 1)? delta : (0-delta));
			
			robot->vx = vx;
			robot->delta = delta;
			
			model_contorl(robot->vx,robot->delta,M1,M2,M3); //运动模型
			
			break;
		}
		case read_speed_sign: // 发送速度给上位机
		{
			send_code(&huart1,read_speed_sign,*M1,*M2,*M3);
			break;
		}
		case motor_stop: // 是否堵转
		{
			send_code(&huart1,read_speed_sign,*M1,*M2,*M3);
			break;
		}
	}
	
}

