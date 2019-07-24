/*
 * eeprom.h
 *
 *  Created on: 2019Äê7ÔÂ24ÈÕ
 *      Author: dell
 */

#ifndef EEPROM_H_
#define EEPROM_H_

void Flash_write(uint32_t wr_addr, void *buffer, uint16_t wr_len);
void FLASH_Read(uint32_t addr, void *buffer, uint16_t len);


#endif /* EEPROM_H_ */
