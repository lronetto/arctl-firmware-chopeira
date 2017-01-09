/*
 * xbee.h
 *
 *  Created on: 14/08/2015
 *      Author: leandro
 */

#ifndef XBEE_H_
#define XBEE_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "fila.h"

#define XBEE_TRANSMIT_REQUEST	0x10
#define XBEE_CMD				0x08
#define XBEE_CMDAT				0x88
#define XBEE_CMDATRT			0x17
#define XBEE_CMDATRR			0x97
#define XBEE_RECEIVE_PACKET 	0x90
#define XBEE_TRANSMIT_STATUS	0x8B
#define XBEE_OFFSET_PAYLOAD		0x0F
#define XBEE_OFFSET_SOURCE		0x04
#define XBEE_STATUS				0x8A

typedef struct{
	uint8_t xbee_coord[8];
	uint8_t type;
	uint8_t Address[8];
	uint8_t addr_cord[8];
	uint8_t source_Address[8];
	uint8_t payload[200];
	uint8_t buf[200];
	uint8_t size,id,start;
	uint8_t cnt,tam;

}xbee_t;

void xbee_reciver(xbee_t *xbee);
void xbee_SendData(xbee_t *xbee,uint8_t *address,uint8_t *data,uint8_t size);
void xbee_cmdAT(xbee_t *xbee,uint8_t *data,uint8_t size);
void xbee_usart(TTFila *fila,uint8_t data);

#endif /* XBEE_H_ */
