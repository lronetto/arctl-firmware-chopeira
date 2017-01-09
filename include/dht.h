/*
 * dht11.h
 *
 *  Created on: 29/08/2013
 *      Author: leandro
 */

#ifndef DHT_H_
#define DHT_H_

#include "stm32f10x.h"

typedef struct {
	GPIO_TypeDef* GPIO;
	GPIO_InitTypeDef GPIO_conf;
	int flag;
	float temp;
	float humid;
	uint8_t *addr;
	}dht_TypeDef;


	void dht_init(dht_TypeDef *dht,GPIO_TypeDef* GPIO,uint32_t GPIO_Pin);
	void dht_read(dht_TypeDef *dht);

#endif /* DHT11_H_ */
