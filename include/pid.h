/*
 * pid.h
 *
 *  Created on: 20 de jun de 2016
 *      Author: Leandro
 */

#ifndef INCLUDE_PID_H_
#define INCLUDE_PID_H_
#include "stm32f10x.h"

typedef struct{
	float Kp,Ki,Kd;
	float erro_ant;
	float SunInt;
	float max;
	float min;
	float out;
	int type; ///refrigera =1
	uint32_t startt;
}PID_T;


void pid_init(PID_T *pid,float Kp,float Ki,float Kd,float min,float max,int type);
void pid_process(PID_T *pid,float SetPoint,float in);

#endif /* INCLUDE_PID_H_ */

