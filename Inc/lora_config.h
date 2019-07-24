/*
 * lora_config.h
 *
 *  Created on: 2019��7��22��
 *      Author: dell
 */

#ifndef LORA_CONFIG_H_
#define LORA_CONFIG_H_
#include "stdint.h"

typedef struct
{
	uint8_t dev_eui[8];
	uint8_t app_eui[8];
	uint8_t app_key[16];
}lora_config_t;


int read_config(int argc , char * argv[]);
int set_config(int argc , char * argv[]);

extern lora_config_t lora_config;

#endif /* LORA_CONFIG_H_ */
