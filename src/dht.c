/*
 * dht11.c
 *
 *  Created on: 29/08/2013
 *      Author: leandro
 */
#include "stm32f10x.h"
#include <stdio.h>
#include <conf.h>
#include "dht.h"

void dht_reset(dht_TypeDef *dht);
void dht_mode(dht_TypeDef *dht,GPIOMode_TypeDef mode);

void dht_init(dht_TypeDef *dht,GPIO_TypeDef* GPIO,uint32_t GPIO_Pin){

	GPIO_InitTypeDef GPIO_conf;
	(*dht).GPIO=GPIO;
	(*dht).flag=0;

	GPIO_conf.GPIO_Pin = GPIO_Pin;
  	GPIO_conf.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_conf.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIO, &GPIO_conf);
	(*dht).GPIO_conf=GPIO_conf;
	//dht_reset(dht);
}
void dht_mode(dht_TypeDef *dht,GPIOMode_TypeDef mode){
	dht->GPIO_conf.GPIO_Mode=mode;
	GPIO_Init(dht->GPIO, &dht->GPIO_conf);
	}
void dht_reset(dht_TypeDef *dht){
	dht_mode(dht,GPIO_Mode_Out_PP);
	dht->GPIO->ODR&=~dht->GPIO_conf.GPIO_Pin;
	Delay_us(1000);
	dht_mode(dht,GPIO_Mode_IN_FLOATING);
}
void dht_read(dht_TypeDef *dht){
	uint8_t byte[6],i=0,j=0;

	uint32_t start,t;
	memset(byte,0,sizeof(byte));
	//__disable_irq();
	dht_reset(dht);
	while(!(dht->GPIO->IDR & dht->GPIO_conf.GPIO_Pin));
	while((dht->GPIO->IDR & dht->GPIO_conf.GPIO_Pin));
	while(!(dht->GPIO->IDR & dht->GPIO_conf.GPIO_Pin));
	while((dht->GPIO->IDR & dht->GPIO_conf.GPIO_Pin));
	for(j=0;j<5;j++){
		for(i=0;i<8;i++){
			while(!(dht->GPIO->IDR & dht->GPIO_conf.GPIO_Pin));
			start=TIM3->CNT;
			while((dht->GPIO->IDR & dht->GPIO_conf.GPIO_Pin));
			//t=TIM3->CNT-start;
			if((TIM3->CNT-start)>40) byte[j]|=(1<<(7-i));
			}
		}

	if(byte[4]==((byte[0] + byte[1] + byte[2] + byte[3]) & 0xFF)){
		dht->humid=(byte[0]*256+byte[1])*0.1;
		dht->temp=((byte[2] & 0x7F)*256+byte[3])*0.1;
		if(byte[2] & 0x80) dht->temp*=-1;
		}
	//printf("temp=%f humid=%f\n\r",dht->temp,dht->humid);



}
