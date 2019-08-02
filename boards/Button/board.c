/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "stm32l0xx.h"
#include "utilities.h"
#include "gpio.h"
//#include "adc.h"
//#include "spi.h"
//#include "i2c.h"
//#include "uart.h"
#include "timer.h"
#include "board-config.h"
#include "rtc-board.h"
#include "sx126x-board.h"
#include "board.h"
#include "lora_config.h"
#include "eeprom.h"

/*!
 * Unique Devices IDs register set ( STM32L0xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * LED GPIO pins objects
 */
Gpio_t Led1;
Gpio_t Led2;
Gpio_t Led3;

Gpio_t Led4;
Gpio_t Led5;


TimerEvent_t Key1_timer;
volatile unsigned int key_holdon_ms=0;
volatile unsigned char key_fall_flag=0;
volatile unsigned char key_short_down=0;
volatile unsigned char key_long_down=0;



void Key1_timer_Callback()
{
	if(key_fall_flag==1)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==0)
		{
			if(key_holdon_ms <= 1500)
			{
				key_holdon_ms+=100;
				TimerStart(&Key1_timer);     //开启按键定时器

			}
			else //按键按下到2000ms就判断长按时间成立，生成长按标志
			{
				key_holdon_ms = 0;
				key_short_down=0;//清短按键标志
				key_long_down = 1;//长按键标志置位
				key_fall_flag = 0;//清按键按下标志
				TimerStop(&Key1_timer);
				key_holdon_ms=0;

			}
		}
		else
		{
//
				key_holdon_ms=0;
				key_short_down=1;
				key_long_down =0;
				key_fall_flag=0;
				TimerStop(&Key1_timer);
				key_holdon_ms=0;



		}
	}

	//printf("key_holdon_ms	%dms\r\n",key_holdon_ms);





}


//}
/*
 * MCU objects
 */
//Uart_t Uart2;

/*!
 * Initializes the unused GPIO to a know status
 */


/*!
 * System Clock Configuration
 */


/*!
 * Used to measure and calibrate the system wake-up time from STOP mode
 */
//static void CalibrateSystemWakeupTime( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */


/*!
 * Timer used at first boot to calibrate the SystemWakeupTime
 */
//static TimerEvent_t CalibrateSystemWakeupTimeTimer;

/*!
 * Flag to indicate if the MCU is Initialized
 */
//static bool McuInitialized = false;

/*!
 * Flag used to indicate if board is powered from the USB
 */
static bool UsbIsConnected = false;

/*!
 * UART2 FIFO buffers size
 */

/*!
 * Flag to indicate if the SystemWakeupTime is Calibrated
 */
//static bool SystemWakeupTimeCalibrated = false;

/*!
 * Callback indicating the end of the system wake-up time calibration
 */
//static void OnCalibrateSystemWakeupTimeTimerEvent( void )
//{
//    SystemWakeupTimeCalibrated = true;
//}

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void BoardInitPeriph( void )
{

}

void BoardInitMcu( void )
{
	    HAL_Init();
		SystemClock_Config();

		MX_GPIO_Init();
		LED_Init();     //加了LED可以正常join
		LED_ALL_ON();
		HAL_Delay(200);
		LED_ALL_OFF();
		HAL_Delay(200);
		LED_ALL_ON();
		HAL_Delay(200);
		LED_ALL_OFF();
		MX_USART2_UART_Init();

		MX_SPI1_Init();
		SX126xIoInit();
		KeyCallbackInit();
		RtcInit();

		TimerInit(&Key1_timer,Key1_timer_Callback);
		TimerSetValue(&Key1_timer,200);
//		TimerStart(&Key1_timer);
		FLASH_Read(FLASH_USER_START_ADDR, &lora_config, sizeof( lora_config_t));  //注意第二个参数的指针

//	    HAL_DBGMCU_EnableDBGSleepMode( );
//		GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//      GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//      GpioInit( &Led3, LED_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );


}





void LED_Cycle()
{
	GpioWrite(&Led1,0);
	GpioWrite(&Led2,1);
	GpioWrite(&Led3,1);
	HAL_Delay(60);
	GpioWrite(&Led1,1);
	GpioWrite(&Led2,0);
	GpioWrite(&Led3,1);
	HAL_Delay(60);
	GpioWrite(&Led1,1);
	GpioWrite(&Led2,1);
	GpioWrite(&Led3,0);
	HAL_Delay(60);

}

void LED_Init()
{
	GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
	GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
	GpioInit( &Led3, LED_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );

	GpioInit( &Led4, LED_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
	GpioInit( &Led5, LED_5, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
}

void LED_ALL_ON()
{
	LED1_State(0);
	LED2_State(0);
	LED3_State(0);
	LED4_State(0);
	//LED5_State(0);
}

void LED_ALL_OFF()
{
	LED1_State(1);
	LED2_State(1);
	LED3_State(1);
	LED4_State(1);
	//LED5_State(1);
}

void LED_RED()
{
	GpioInit( &Led5, LED_5, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
	LED5_State(0);
	HAL_Delay(150);
	LED5_State(1);
	HAL_Delay(150);
	LED5_State(0);
	HAL_Delay(150);
	LED5_State(1);


}

void LED_Indication()
{
	GpioInit( &Led4, PA_14, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
	LED4_State(0);
	HAL_Delay(150);
	LED4_State(1);
	HAL_Delay(150);
	LED4_State(0);
	HAL_Delay(150);
	LED4_State(1);
}


void LED1_State(uint32_t state)
{
	GpioWrite(&Led1,state);
}

void LED2_State(uint32_t state)
{
	GpioWrite(&Led2,state);
}

void LED3_State(uint32_t state)
{
	GpioWrite(&Led3,state);
}

void LED4_State(uint32_t state)
{
	GpioWrite(&Led4,state);
}

void LED5_State(uint32_t state)
{
	GpioWrite(&Led5,state);
}


void BoardResetMcu( void )
{

    BoardDisableIrq( );

    //Restart system
    NVIC_SystemReset( );

}

void BoardDeInitMcu( void )
{


	HAL_SPI_DeInit(&hspi1);
	LED_Init();
	LED1_State(1);
	LED2_State(1);
	LED3_State(1);
	LED4_State(1);
	LED5_State(1);

	//__HAL_RCC_DMA1_CLK_DISABLE();
	//SX126xIoDeInit();

}




uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

uint16_t BoardBatteryMeasureVolage( void )
{
    return 0;
}

uint32_t BoardGetBatteryVoltage( void )
{
    return 0;
}

uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}

//static void BoardUnusedIoInit( void )
//{
//    HAL_DBGMCU_EnableDBGSleepMode( );
//    HAL_DBGMCU_EnableDBGStopMode( );
//    HAL_DBGMCU_EnableDBGStandbyMode( );
//}

//void SystemClockConfig( void )
//{
//    RCC_OscInitTypeDef RCC_OscInitStruct;
//    RCC_ClkInitTypeDef RCC_ClkInitStruct;
//    RCC_PeriphCLKInitTypeDef PeriphClkInit;
//
//    __HAL_RCC_PWR_CLK_ENABLE( );
//
//    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );
//
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
//    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
//    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
//    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
//    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
//    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_6;
//    RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;
//    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
//    {
//        assert_param( FAIL );
//    }
//
//    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
//    {
//        assert_param( FAIL );
//    }
//
//    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
//    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
//    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
//    {
//        assert_param( FAIL );
//    }
//
//    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );
//
//    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );
//
//    // SysTick_IRQn interrupt configuration
//    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
//}

//void CalibrateSystemWakeupTime( void )
//{
//    if( SystemWakeupTimeCalibrated == false )
//    {
//        TimerInit( &CalibrateSystemWakeupTimeTimer, OnCalibrateSystemWakeupTimeTimerEvent );
//        TimerSetValue( &CalibrateSystemWakeupTimeTimer, 1000 );
//        TimerStart( &CalibrateSystemWakeupTimeTimer );
//        while( SystemWakeupTimeCalibrated == false )
//        {
//            TimerLowPowerHandler( );
//        }
//    }
//}

//void SystemClockReConfig( void )
//{
//    __HAL_RCC_PWR_CLK_ENABLE( );
//    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );
//
//    /* Enable HSI */
//    __HAL_RCC_HSI_CONFIG( RCC_HSI_ON );
//
//    /* Wait till HSE is ready */
//    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
//    {
//    }
//
//    /* Enable PLL */
//    __HAL_RCC_PLL_ENABLE( );
//
//    /* Wait till PLL is ready */
//    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
//    {
//    }
//
//    /* Select PLL as system clock source */
//    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );
//
//    /* Wait till PLL is used as system clock source */
//    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
//    {
//    }
//}

//void SysTick_Handler( void )
//{
//    HAL_IncTick( );
//    HAL_SYSTICK_IRQHandler( );
//}

uint8_t GetBoardPowerSource( void )
{
    if( UsbIsConnected == false )
    {
        return BATTERY_POWER;
    }
    else
    {
        return USB_POWER;
    }
}




