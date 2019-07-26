/*
 * command.h
 *
 *  Created on: 2019Äê7ÔÂ22ÈÕ
 *      Author: dell
 */

#ifndef COMMAND_H_
#define COMMAND_H_

void CMD_Process( unsigned char* rxChar);

char  AsciiToHex(unsigned char * pAscii, unsigned char * pHex, int nLen);
#define MAX_ARGV 20




#endif /* COMMAND_H_ */
