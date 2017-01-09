/*
 * pid.c
 *
 *  Created on: 20 de jun de 2016
 *      Author: Leandro
 */
#include "pid.h"
#include "stm32f10x.h"

void pid_init(PID_T *pid,float Kp,float Ki,float Kd,float min,float max,int type){
	pid->Kp=Kp;
	pid->Ki=Ki;
	pid->Kd=Kd;
	pid->SunInt=0;
	pid->erro_ant=0;
	pid->min=min;
	pid->max=max;
	pid->type=type;
	pid->startt=TIM2->CNT;
}
void pid_process(PID_T *pid,float SetPoint,float in){
	float saida;
	float erro=SetPoint-in;
	if(pid->type)
		erro=in-SetPoint;

	float dt=(millis()-pid->startt)/1000.0;

	pid->SunInt+=(pid->Ki*erro)*(float)dt;
	if(pid->SunInt > pid->max) pid->SunInt=pid->max;
	if(pid->SunInt < pid->min) pid->SunInt=pid->min;

	saida=-pid->Kd*(erro - pid->erro_ant)/(float)dt;

	saida+=pid->Kp*erro;

	saida+=pid->SunInt;

	printf("pid in=%02.4f out=%02.4f erro=%02.4f kp=%02.4f ki=%02.4f kd=%02.4f dt=%2.4f\r\n",in,saida,erro,pid->Kp*erro,pid->SunInt,pid->Kd*(erro - pid->erro_ant)*(float)dt,dt);

	pid->erro_ant=erro;


	if(saida > pid->max) saida=pid->max;

	if(saida < pid->min) saida=pid->min;

	pid->startt=millis();

	pid->out=saida;

}
