

#include "stm32f10x_conf.h"
#include "pid.h"
//#include "var.h"

typedef union{
	uint16_t var;
	struct{
		uint8_t b[2];
	}byte;
}uint16byte_T;
typedef union{
	float val;
	struct{
		char b[4];
    	}byte;
}floatbyte_T;

typedef struct{
	uint8_t status;
	uint8_t id_disp;
	uint8_t id_sys;
	uint16byte_T tempo;
	floatbyte_T setpoint,hist;
	floatbyte_T p,i,d;
	uint8_t tipo;
	float pot;
}Disp_T;


typedef struct {
	Disp_T disp;
}EE_t;

void EE_Write(EE_t EEP);
void EE_Read(EE_t *EEP);
