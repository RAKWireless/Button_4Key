/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "stdio.h"
#include "lora_config.h"
#include "timer.h"
#include "board.h"
/* USER CODE BEGIN 0 */
extern  unsigned char key1_fall_flag;  //按键按下标志位
extern  unsigned char key2_fall_flag;


/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
extern TimerEvent_t Key_timer;


/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin : PA9 */
	  GPIO_InitStruct.Pin = GPIO_PIN_9;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;   //下降沿中断   注意
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA10 */
	  GPIO_InitStruct.Pin = GPIO_PIN_10;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PB6 PB7 */
	  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0,0);
	  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

extern SPI_HandleTypeDef hspi1;
/* USER CODE BEGIN 2 */
void Key1_Fall()
{
	//merStart(&Key1_timer);     //开启按键定时器
	key1_fall_flag=1;
	//L_Delay(100;
	TimerStart(&Key_timer);     //开启按键定时器

//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	GPIO_InitStruct.Pin = GPIO_PIN_9;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


//
//	HAL_SPI_DeInit(&hspi1);
//	LED_Init();
//	LED1_State(0);
//	HAL_Delay(300);
//	LED1_State(1);
//	MX_SPI1_Init();
//
//	unsigned char str[1]={0x01};
//	lora_send(1,str);

}

void Key2_Fall()
{

	key2_fall_flag=1;
	//L_Delay(100;
	TimerStart(&Key_timer);     //开启按键定时器
//	printf("KEY2 Fall\r\n");
//	HAL_SPI_DeInit(&hspi1);
//	LED_Init();
//	LED2_State(0);
//	HAL_Delay(300);
//	LED2_State(1);
//	MX_SPI1_Init();
//
//	unsigned char str[1]={0x02};
//	lora_send(2,str);
}
void Key3_Fall()
{
	printf("KEY3 Fall\r\n");
	HAL_SPI_DeInit(&hspi1);
	LED_Init();
	LED3_State(0);
	HAL_Delay(300);
	LED3_State(1);
	MX_SPI1_Init();

	unsigned char str[1]={0x03};
	lora_send(3,str);
}
void Key4_Fall()
{
	printf("KEY4 Fall\r\n");
	HAL_SPI_DeInit(&hspi1);
	LED_Init();
	LED4_State(0);
	HAL_Delay(300);
	LED4_State(1);
	MX_SPI1_Init();

	unsigned char str[1]={0x04};
	lora_send(4,str);
}


/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
