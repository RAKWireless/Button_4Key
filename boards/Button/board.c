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


extern void Adc_timer_Callback(void);

TimerEvent_t Key_timer;
TimerEvent_t Adc_timer;

extern ADC_HandleTypeDef hadc;

volatile unsigned int key_holdon_ms=0;
volatile unsigned char key1_fall_flag=0;
volatile unsigned char key2_fall_flag=0;

volatile unsigned char key1_short_down=0;
volatile unsigned char key1_long_down=0;

volatile unsigned char key2_short_down=0;
volatile unsigned char key2_long_down=0;
TimerEvent_t Batteryalarm_timer;
void Batteryalarm_timer_callback()
{
	 static unsigned char state= 0;
	 state = !state;
	 LED5_State(!state);
	 TimerStart(&Batteryalarm_timer);
}


void Key_timer_Callback()
{
	if(key1_fall_flag==1)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9)==0)
		{
			if(key_holdon_ms <= 1500)
			{
				key_holdon_ms+=100;
				TimerStart(&Key_timer);     //开启按键定时器

			}
			else //按键按下到2000ms就判断长按时间成立，生成长按标志
			{
				key_holdon_ms = 0;
				key1_short_down=0;//清短按键标志
				key1_long_down = 1;//长按键标志置位
				key1_fall_flag = 0;//清按键按下标志
				TimerStop(&Key_timer);
				key_holdon_ms=0;

			}
		}
		else
		{
//
				key_holdon_ms=0;
				key1_short_down=1;
				key1_long_down =0;
				key1_fall_flag=0;
				TimerStop(&Key_timer);
				key_holdon_ms=0;



		}
	}



	if(key2_fall_flag==1)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==0)
		{
			if(key_holdon_ms <= 1500)
			{
				key_holdon_ms+=100;
				TimerStart(&Key_timer);     //开启按键定时器

			}
			else //按键按下到2000ms就判断长按时间成立，生成长按标志
			{
				key_holdon_ms = 0;
				key2_short_down=0;//清短按键标志
				key2_long_down = 1;//长按键标志置位
				key2_fall_flag = 0;//清按键按下标志
				TimerStop(&Key_timer);
				key_holdon_ms=0;

			}
		}
		else
		{
//
				key_holdon_ms=0;
				key2_short_down=1;
				key2_long_down =0;
				key2_fall_flag=0;
				TimerStop(&Key_timer);
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

		TimerInit(&Key_timer,Key_timer_Callback);
		TimerSetValue(&Key_timer,200);

		FLASH_Read(FLASH_USER_START_ADDR, &lora_config, sizeof( lora_config_t));

		TimerInit(&Adc_timer,Adc_timer_Callback);
		TimerSetValue(&Adc_timer,900*1000);
		TimerStart(&Adc_timer);
		TimerInit(&Batteryalarm_timer,Batteryalarm_timer_callback);
		TimerSetValue(&Batteryalarm_timer,1000);

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

}

void LED_ALL_OFF()
{
	LED1_State(1);
	LED2_State(1);
	LED3_State(1);
	LED4_State(1);
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
//	LED_Init();

	LED1_State(1);
	LED2_State(1);
	LED3_State(1);
	LED4_State(1);
//	LED5_State(1);

	//__HAL_RCC_DMA1_CLK_DISABLE();
	//SX126xIoDeInit();

}




uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
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




