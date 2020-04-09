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
#define VREFINT_CAL                         *(__IO uint16_t *)(0x1FF80078)
unsigned char len;
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
extern void CMD_Process( unsigned char* rxChar);
void MX_ADC_Init(void);
__IO float V=0;
void Get_Bat();
__IO uint16_t ADC_Value[2]={0};
extern TimerEvent_t Adc_timer;
extern TimerEvent_t Batteryalarm_timer;

void Adc_timer_Callback()
{
	Get_Bat();
	HAL_Delay(5);
	V =  3*(float) VREFINT_CAL /(float)(ADC_Value[0]);
	//printf("Voltage = %d\r\n",(int)(V*100));
	HAL_ADC_DeInit(&hadc);

	//printf("Adc_timer_Callback\r\n");
	if(V<2.8)
	{
		//printf("Battery alarm\r\n");
		TimerStart(&Batteryalarm_timer);
	}
	else
	{
		TimerStop(&Batteryalarm_timer);
		LED5_State(1);
	}
	TimerStart(&Adc_timer);
}

int main(void)
{
	//SCB->VTOR = FLASH_BASE | 0x2800;//
	BoardInitMcu();


	printf("Button Application (Version 1.0.3)\r\n");
	InitLora();

	while(1)
	{

		if(key1_long_down == 1)
		{
			key1_long_down=0;
			Lower_Power=(!Lower_Power);
			if(!Lower_Power)
			MX_USART2_UART_Init();
			//printf("Lower_Power mode %d\r\n",Lower_Power);
		}
		if(key1_short_down == 1)
		{
			key1_short_down=0;
			printf("KEY1 Fall\r\n");
			HAL_SPI_DeInit(&hspi1);
			LED_Init();
			LED1_State(0);
			HAL_Delay(300);
			LED1_State(1);
			MX_SPI1_Init();

			//unsigned char str[1]={0x01};
			unsigned char str[1]={0x01};
			lora_send(1,str);

		}

		if(key2_long_down == 1)
		{
			key2_long_down=0;
			HAL_NVIC_SystemReset();

		}
		if(key2_short_down == 1)
		{
			key2_short_down=0;
			printf("KEY2 Fall\r\n");
			HAL_SPI_DeInit(&hspi1);
			LED_Init();
			LED2_State(0);
			HAL_Delay(300);
			LED2_State(1);
			MX_SPI1_Init();

			unsigned char str[1]={0x02};

			lora_send(2,str);

		}


		if(USART_RX_STA&0x8000)
		{
			HAL_SPI_DeInit(&hspi1);
			MX_SPI1_Init();

			len=USART_RX_STA&0x3fff;//
			USART_RX_BUF[len]=0;
			CMD_Process(USART_RX_BUF);
			memset(USART_RX_BUF,0,200);

			USART_RX_STA=0;
		}

		if(Lower_Power==true)
		{


		 HAL_UART_DeInit(&huart2);
		 BoardDeInitMcu();
		 SystemPower_Config();
		 HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		}
		else
		{
		 LED5_State(0);
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

  	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }


  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
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

  //__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);

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



}

void MX_DMA_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  //HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}


//static void MX_GPIO_Init(void)
//{
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//
//}


void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }


  /** Configure for the selected ADC regular channel to be converted.
  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}



void Get_Bat()
{
	MX_DMA_Init();
	MX_ADC_Init();
	if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) !=  HAL_OK)
	{
    Error_Handler();
 	}
	//HAL_ADC_Stop_DMA(&hadc,(uint32_t *)ADC_Value,1);
	HAL_ADC_Start_DMA(&hadc,(uint32_t *)ADC_Value,1);

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
	printf("%s	%s	%d\r\n",__FILE__,__func__,__LINE__);
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
