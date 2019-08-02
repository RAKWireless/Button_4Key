/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
//DR0 进入txdone 不要立即切换模式，可能导致发射异常，需要验证
//开机谜之延时，否者串口空闲中断工作异常


#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"


#include "stdio.h"
#include "board.h"
#include "sx126x-board.h"
#include "delay.h"
#include "sx126x.h"
#include "timer.h"
#include "rtc-board.h"
#include "string.h"
#include "stdbool.h"
#include "LoRaMac.h"

#include "utilities.h"
#include "lora_config.h"
#include "gpio-board.h"

volatile bool Lower_Power = 1 ;
//extern unsigned char USART_RX_STA;
//bool temp;
unsigned char len;

extern void CMD_Process( unsigned char* rxChar);

//TimerEvent_t Led_Red;

//void Led_Red_Callback()
//{
//
//
//	LED5_State(~temp);
//	TimerStart(&Led_Red);
//}

//SWDCLK灯常亮
int main(void)
{
	SCB->VTOR = FLASH_BASE | 0x2800;//
	BoardInitMcu();
	//HAL_Delay(500); //谜之延时


	printf("\r\n======================================================================");
	//printf("\r\n=              (C) COPYRIGHT 2015 STMicroelectronics                 =");
    //printf("\r\n=                                                                    =");
	printf("\r\n=              STM32L0xx Button Application  (Version 1.0.0)         =");
	//printf("\r\n=                                                                    =");
	//printf("\r\n=                                              By  RAK Team          =");
	printf("\r\n======================================================================");
	printf("\r\n\r\n");

	InitLora();





	while(1)
	{

		if(key_long_down == 1)
		{
			key_long_down=0;
			Lower_Power=(!Lower_Power);
			if(!Lower_Power)
			MX_USART2_UART_Init();
			printf("Lower_Power mode %d\r\n",Lower_Power);
		}
		if(key_short_down == 1)
		{
			key_short_down=0;
			printf("KEY1 Fall\r\n");
			HAL_SPI_DeInit(&hspi1);
			LED_Init();
			LED1_State(0);
			HAL_Delay(300);
			LED1_State(1);
			MX_SPI1_Init();

			unsigned char str[1]={0x01};
			lora_send(1,str);

		}

		if(USART_RX_STA&0x8000)
		{
			HAL_SPI_DeInit(&hspi1);
			MX_SPI1_Init();

			len=USART_RX_STA&0x3fff;//
			USART_RX_BUF[len]=0;
			CMD_Process(USART_RX_BUF);
			memset(USART_RX_BUF,0,200);

			//HAL_UART_Transmit(&huart2,(uint8_t*)USART_RX_BUF,len,1000);	//
			//while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=SET);
			//printf("\r\n\r\n");//
			USART_RX_STA=0;
		}

       //HAL_UART_MspDeInit(&huart2);
       //Lp_uart();
		if(Lower_Power==true)
		{
//		 LED5_State(1);
//		 TimerStop(&Led_Red);
//		 HAL_Delay(3000);
//		 HAL_UART_MspDeInit(&huart2);   //关闭串口
		 HAL_UART_DeInit(&huart2);
		 //UnLp_uart();
		 BoardDeInitMcu();
		 //HAL_Delay(5000);
		 SystemPower_Config();
		 HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

		}
		else
		{
			LED5_State(0);

			//TimerStart(&Led_Red);
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
  {
	  assert_param( FAIL );
  }

}

/* USER CODE BEGIN 4 */
static void SystemPower_Config(void)
{


  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();
  //HAL_UARTEx_EnableStopMode(&huart2);


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
	//printf("%s	%s	%d\r\n",__FILE__,__func__,__LINE__);
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
