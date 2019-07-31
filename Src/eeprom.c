/*
 * eeprom.c
 *
 *  Created on: 2019Äê7ÔÂ24ÈÕ
 *      Author: dell
 */
#include "main.h"
#include "lora_config.h"
#include "stdio.h"
#include "eeprom.h"

static void _ENABLE_FLASH(void)
{
    HAL_FLASH_Unlock();
}

static void _DISABLE_FLASH(void)
{
    HAL_FLASH_Lock();
}


void Flash_write(uint32_t wr_addr, void *buffer, uint16_t wr_len)
{
    int i;

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t *wr_data = buffer;

    if (wr_addr % 4 != 0 || wr_len%4 != 0) {
    	//printf("wr_len	%d\r\n",wr_len);
        while(1);
    }

    _ENABLE_FLASH();

    for (i = 0; i < wr_len/4; i++)
    {

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, wr_addr, *wr_data);
        wr_data++;
        wr_addr += 4;
    }
    _DISABLE_FLASH();
}


void FLASH_Read(uint32_t addr, void *buffer, uint16_t len)
{
       memcpy(buffer, (void *)addr, len);
       //__IO uint32_t
       //  memcpy(buffer, (__IO uint32_t  *)addr, len);
}







