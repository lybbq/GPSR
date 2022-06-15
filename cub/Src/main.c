/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include "send.h"
#include "Motor.h"
#include "encode.h"
#include "string.h"
#include "model.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//typedef enum {FALSE = 0,TRUE = 1} bool;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Press_begin HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==1 // PE4 is Low  beginning
#define code_init_value 0x7Fff
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint32_t cnt_num = 0;
	uint32_t cnt_time  = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
	// Motor_Fwd(Motor_1);
	// while(Press_begin); // begining
	/*        open Encode       */
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //Private define
	HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);//Private define
	
	__HAL_TIM_SET_COUNTER(Motor_1.htim_encode,code_init_value); //reset value to oxff
	HAL_TIM_Encoder_Start(Motor_1.htim_encode, TIM_CHANNEL_ALL);// Open Channel 4
	__HAL_TIM_SET_COUNTER(Motor_2.htim_encode,code_init_value); //reset value to oxff
	HAL_TIM_Encoder_Start(Motor_2.htim_encode, TIM_CHANNEL_ALL);// Open Channel 4
	__HAL_TIM_SET_COUNTER(Motor_3.htim_encode,code_init_value); //reset value to oxff
	HAL_TIM_Encoder_Start(Motor_3.htim_encode, TIM_CHANNEL_ALL);// Open Channel 4
	
	HAL_TIM_Base_Start_IT(&htim6); // open time6 
	
	init_pid(&Motor_1.pid,0.3,0.1,0);
	init_pid(&Motor_2.pid,0.3,0.1,0);
	init_pid(&Motor_3.pid,0.3,0.1,0);
	// conf - plus and min_speed  0-200 ,200-400 ,400-1000 x =plus y = speed
	// Motor_1.pid(0.5,0.1,0);
	  /* USER CODE BEGIN WHILE */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {		//HAL_UART_Transmit(&huart1,&recv_end_flag,1,2000);
		// run 
		
		if(recv_end_flag)
		{
			 recv_end_flag = 0;
			 
			 // Motor_2.puls = 200;
			 // Motor_control(&Motor_2,Motor_2.puls);
			 Motor_2.min_speed_pre  = 0.5 * arg_max_to_min;
			 // Hanlde_receive(rx_buffer,rx_len,&robot,&Motor_1,&Motor_2,&Motor_3); // 处理上位机信息 包括上位机读取速度，写入速度，读取异常状态
			 memset(rx_buffer,0,rx_len);
			 HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t abs_uint32_t(uint32_t value1,uint32_t value2)
{
	if(value1 > value2)
		return value1 - value2;
	else
		return value2 - value1;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // 外部中断
{
		uint32_t value ;
		uint8_t  tx = 0;
		// uint32_t temp_value = code_init_value;
		
   if(GPIO_Pin == GPIO_PIN_13)  
   {
			// Usart_Unin32(&huart1,(uint32_t)tx );
		 //uint32_t temp_value = (uint32_t)(__HAL_TIM_GET_COUNTER(Motor_2.htim_encode));
		 //Usart_Unin32(&huart1,temp_value);
		 //__HAL_TIM_SET_COUNTER(Motor_2.htim_encode,0);
			/*temp_value = (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2));
		 //Usart_Unin32(&huart1,temp_value);
		  value = abs(code_init_value-temp_value);
			cnt_num += value;
		  Usart_Unin32(&huart1,value);
		  __HAL_TIM_SET_COUNTER(&htim2,code_init_value); //重置编码器值*/
		 // Usart_Unin32(&huart1,cnt_num);
		 
   }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //定时器
{
	/*           定时器目前设为10ms一次传输数据                   */
		float error1 = Motor_1.min_speed_pre - Motor_1.min_speed_real;
		float error2 = Motor_2.min_speed_pre - Motor_2.min_speed_real;
		float error3 = Motor_3.min_speed_pre - Motor_3.min_speed_real;
	  if(htim->Instance == TIM6)                 
   {
		 Encode_Motor(&Motor_1,code_init_value);
		 Encode_Motor(&Motor_2,code_init_value);
		 Encode_Motor(&Motor_3,code_init_value);
		 // 传输
		 cnt_time++;
   }
		 // pid control
	 if(abs(error1) > 3)
		{
				Motor_1.set_speed += pid_regultion(&Motor_1.pid,error1);
				Motor_1.puls = speed_to_puls(Motor_1.set_speed);
				Motor_control(&Motor_1,Motor_1.puls);
		}
	 if(abs(error2) > 3)
		{
				Motor_2.set_speed += pid_regultion(&Motor_2.pid,error2);
				if(Motor_2.set_speed > 144) Motor_2.set_speed = 144;
				Motor_2.puls = speed_to_puls(Motor_2.set_speed);
				Motor_control(&Motor_2,Motor_2.puls);			
			
		}
	 if(abs(error3) > 3)
		{
				Motor_3.set_speed += pid_regultion(&Motor_3.pid,error3);
				Motor_3.puls = speed_to_puls(Motor_3.set_speed);
				Motor_control(&Motor_3,Motor_3.puls);
		}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
