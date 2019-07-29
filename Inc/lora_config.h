/*
 * lora_config.h
 *
 *  Created on: 2019Äê7ÔÂ22ÈÕ
 *      Author: dell
 */

#ifndef LORA_CONFIG_H_
#define LORA_CONFIG_H_
#include "stdint.h"
#define FLASH_USER_START_ADDR   (unsigned int)0x8080000

typedef enum {
	OATT,
	ABP
}join_states;

typedef enum {
  CFG_READ,
  CFG_WRITE
}cfg_op;

typedef struct
{
	uint8_t dev_eui[8];
	uint8_t app_eui[8];
	uint8_t app_key[16];

	uint8_t dev_addr[4];
	uint8_t apps_key[16];
	uint8_t nwks_key[16];
	char temp1;
	char temp2;
	char temp3;
    join_states join_state;


}lora_config_t;


int read_config(int argc , char * argv[]);
int set_config(int argc , char * argv[]);

void InitLora(void);
void lora_join(int argc, char *argv[]);

void lora_send(int port,const unsigned char* Appdata);



extern lora_config_t lora_config;

#endif /* LORA_CONFIG_H_ */
