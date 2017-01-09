/*
 * xbee.c
 *
 *  Created on: 14/08/2015
 *      Author: leandro
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "xbee.h"
#include "conf.h"
#include "stm32f10x.h"
void xbee_get_address(xbee_t *xbee);
uint16_t cnt=0;
uint8_t flag_ser=2;
uint8_t tam,buf[100],ind;
void xbee_reciver(xbee_t *xbee){
	int tam=xbee->buf[1] | xbee->buf[2];
	int i,sum=0;
	for(i=3;i<tam+3;i++){
		sum+=xbee->buf[i];
		//printf("0x%02X ",data[i]);
		}
	sum&=0xFF;
	sum=0xFF-sum;
	//printf("checksum: 0x%02X calc:0x%02X",data[tam+3],sum);
	if(xbee->buf[tam+3]==sum){
		xbee->type=xbee->buf[3];
		xbee_get_address(xbee);
	}
	else xbee->type=-1;

}
void xbee_get_address(xbee_t *xbee){
	int i;
	for(i=0;i<8;i++)
		xbee->source_Address[i]=xbee->buf[4+i];
}
void xbee_data(xbee_t *xbee,uint8_t *data,uint8_t size){
	int i;
	for(i=17;i<(size+17);i++){
		xbee->payload[i]=data[i-17];
	}
	//printf("'%s' %d\n",data,size);
	xbee->size=size;
}
void xbee_address(xbee_t *xbee,uint8_t *address){
	int i;
	for(i=0;i<8;i++){
		xbee->Address[i]=address[i];
		//printf("0x%02x ",xbee->Address[i]);
	}
}
uint8_t xbee_checksum(xbee_t *xbee){
	int sum=0,i;
	for(i=3;i<xbee->size+3;i++)
		sum+=xbee->payload[i];
	sum&=0xFF;
	sum=0xFF-sum;
	return sum;
}
void xbee_packet_data(xbee_t *xbee,uint8_t *address,uint8_t *data, uint8_t size){
	int i;
	for(i=0;i<8;i++)
			xbee->payload[5+i]=address[i];
	xbee->payload[13]=0xFF;
	xbee->payload[14]=0xFE;
	xbee->payload[15]=0x00;
	xbee->payload[16]=0x00;
	for(i=17;i<(size+17);i++){
			xbee->payload[i]=data[i-17];
		}
	xbee->size=size+12;
}
void xbee_packet(xbee_t *xbee,uint8_t type){
	int i;
	xbee->size+=2;
	xbee->payload[0]=0x7e;
	xbee->payload[1]=0;
	xbee->payload[2]=xbee->size;
	//tipo
	xbee->payload[3]=type;
	//id
	xbee->payload[4]=0x10;
	xbee->payload[xbee->size+3]=xbee_checksum(xbee);
	//printf("tam %d \n",xbee->size);
}
void xbee_Send(xbee_t *xbee){
	int i;
/*	printf("send ");
	for(i=0;i<xbee->size+4;i++){
		printf("%02X ",xbee->payload[i]);
	}
	printf("\r\n");*/
	usart_send_n(USART2,xbee->payload,xbee->size+4);

}
void xbee_SendData(xbee_t *xbee,uint8_t *address,uint8_t *data,uint8_t size){
	xbee_packet_data(xbee,address,data,size);
	xbee_packet(xbee,XBEE_TRANSMIT_REQUEST);
	xbee_Send(xbee);

}
void xbee_cmdAT(xbee_t *xbee,uint8_t *data,uint8_t size){
	int i;
	for(i=0;i<size;i++)
		xbee->payload[5+i]=data[i];
	xbee->size=size;
	xbee_packet(xbee,XBEE_CMD);
	xbee_Send(xbee);
}
int checksum(uint8_t *buf,int tam){
	int i=0,sum=0;
	sum=0;
	for(i=3;i<tam-1;i++){
		sum+=buf[i];
		}
	sum&=0xFF;
	sum=0xFF-sum;
	if(sum==buf[tam-1]) return 1;
	return 0;
}
void xbee_usart(TTFila *fila,uint8_t data){
	uint8_t flag_c=0,i;
	uint8_t sum;
	switch(flag_ser){
		case 2:
			if(data==0x7e){
				memset(buf,0,sizeof(buf));
				buf[0]=0x7e;
				ind=0;
				flag_ser=1;
				}
			break;
		case 1:
			ind++;
			buf[ind]=data;
			if(ind==2)
				tam=(buf[2] | buf[1]<<8);
			if(ind>=2){
				if(ind>2 & ind==(tam+3)){
					if(checksum(buf,tam+4)){
						fila_Packet(fila,buf,tam+4);
						}
					flag_ser=2;
					}
				}

			break;
		}
}

