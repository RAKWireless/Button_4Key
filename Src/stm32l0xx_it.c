/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "command.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
//extern DMA_HandleTypeDef hdma_usart2_rx;
//extern uint8_t Receive_buff[255];
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */


//UART_HandleTypeDef UART1_Handler; /

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
//   HAL_UART_DMAStop(&huart2);                                                     //停止本次DMA传输
//   uint8_t data_length  = USART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);   //计算接收到的数据长度
//   //Receive_buff[data_length]=0;
////   if(Receive_buff[data_length-1]==0x0A&&Receive_buff[data_length-2]==0x0D)
////   {
//	   //printf("Receive Data(length = %d):",data_length);
////	   Receive_buff[data_length-2] = 0;
////   }
////   else
////   {
//	   Receive_buff[data_length] = 0;
// //  }
//
//
//   printf("Receive Data(length = %d):",data_length);
////
//    HAL_UART_Transmit(&huart2,(uint8_t*)Receive_buff,data_length,0x200);                     //测试函数：将接收到的数据打印出去
////    printf("\r\n");
//
//    if(data_length!=0)
//    CMD_Process(Receive_buff);
//
//    memset(Receive_buff,0,128);                                            //清零接收缓冲区
//    data_length = 0;
//    HAL_UART_Receive_DMA(&huart2, Receive_buff, 128);                    //重启开始DMA传输 每次255字节数据
}

void USER_UART_IRQHandler(UART_HandleTypeDef *huart)
{
//    if(USART2 == huart2.Instance)                                   //判断是否是串口1
//    {
//        if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))   //判断是否是空闲中断
//        {
//            __HAL_UART_CLEAR_IDLEFLAG(&huart2);                     //清楚空闲中断标志（否则会一直不断进入中断）
//
//            USAR_UART_IDLECallback(huart);                          //调用中断处理函数
//        }
//    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		if((USART_RX_STA&0x8000)==0)//
		{
			if(USART_RX_STA&0x4000)//
			{
				if(aRxBuffer[0]!=0x0a)
					USART_RX_STA=0;//
				else
				{
					USART_RX_STA|=0x8000;	//
					//HAL_UART_Transmit(&huart2,(uint8_t*)"A",1,1000);



				}
			}
			else //
			{
				if(aRxBuffer[0]==0x0d)
				{
					USART_RX_STA|=0x4000;
					//HAL_UART_Transmit(&huart2,(uint8_t*)"D",1,1000);
				}
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;
					USART_RX_STA++;
					if(USART_RX_STA>(200))
						USART_RX_STA=0;//
				}
			}
		}



	//printf("%d	%d\r\n",__LINE__,USART_RX_STA);

}



void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);

	HAL_UART_Receive_IT(&huart2, (unsigned char *)aRxBuffer, 1);
//	LED_Init();
//	LED4_State(0);
//	HAL_Delay(500);
//	LED4_State(1);
//	HAL_Delay(200);
//	LED4_State(0);
//	HAL_Delay(500);
//	LED4_State(1);
//	HAL_UART_MspInit(&huart2);

//    HAL_UART_IRQHandler(&huart2);
////    RtcRecoverMcuStatus();
//    //USER_UART_IRQHandler(&huart2);
//    unsigned int  timeout=0;
//    unsigned int  maxDelay=0x1FFFF;
//    timeout=0;
////  printf("%d\r\n",__LINE__);
//    while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)//
//    {
//
////    	timeout++;//
////    	if(timeout>maxDelay) break;
//    }
//    timeout=0;
//    while(HAL_UART_Receive_IT(&huart2, (unsigned char *)aRxBuffer, 1) != HAL_OK)//
//    {
////    	timeout++; //
////    	if(timeout>maxDelay) break;
////    	printf("%d\r\n",__LINE__);
//    }
////    printf("%d\r\n",__LINE__);

//	unsigned char Res;
//	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE)!=RESET))  //½ÓÊÕÖÐ¶Ï(½ÓÊÕµ½µÄÊý¾Ý±ØÐëÊÇ0x0d 0x0a½áÎ²)
//	{
//        HAL_UART_Receive(&huart2,&Res,1,1000);
//		if((USART_RX_STA&0x8000)==0)//½ÓÊÕÎ´Íê³É
//		{
//			if(USART_RX_STA&0x4000)//½ÓÊÕµ½ÁË0x0d
//			{
//				if(Res!=0x0a)USART_RX_STA=0;//½ÓÊÕ´íÎó,ÖØÐÂ¿ªÊ¼
//				else USART_RX_STA|=0x8000;	//½ÓÊÕÍê³ÉÁË
//			}
//			else //»¹Ã»ÊÕµ½0X0D
//			{
//				if(Res==0x0d)USART_RX_STA|=0x4000;
//				else
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
//					USART_RX_STA++;
//					if(USART_RX_STA>(200))
//						USART_RX_STA=0;//½ÓÊÕÊý¾Ý´íÎó,ÖØÐÂ¿ªÊ¼½ÓÊÕ
//				}
//			}
//		}
//	}
//	HAL_UART_IRQHandler(&huart2);


    /* USER CODE END USART1_IRQn 1 */
}


extern DMA_HandleTypeDef hdma_adc;
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc);

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

//void DMA1_Channel4_5_6_7_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 0 */
//
//  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_usart2_rx);
//  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 1 */
//
//  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 1 */
//}
/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
