/*
 * arctl.h
 *
 *  Created on: 27 de nov de 2016
 *      Author: leandro
 */

#ifndef INCLUDE_ARCTL_H_
#define INCLUDE_ARCTL_H_

//escravo envia no inicio
#define ARCTL_INIT			0x01
//escravo envia temperaturas
#define ARCTL_TEMP			0x02
//mestre envia temperatura
#define ARCTL_SET_TEMP		0x03
//inicio do mestre
#define ARCTL_MESTRE		0x04
//mestre envia para o escravo
#define ARCTL_PING			0x05
//escravo manda para o mestre
#define ARCTL_PING1			0x06

#endif /* INCLUDE_ARCTL_H_ */
