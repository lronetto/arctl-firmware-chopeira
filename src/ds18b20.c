// Example of how to use the pcd8544 library, the lcd-driver used in
// Nokia 3310 and others.
//
// Tested on Sparkfun's "Graphic LCD 84x48 - Nokia 5110".
#include "stm32f10x.h"
#include "ds18b20.h"
#include "conf.h"
#include <stdio.h>

//extern void Delay_ms(__IO uint32_t nTime);
//extern void Delay_us(__IO uint32_t nTime);


void Ds18b20_Mode(ds18b20_TypeDef ds,GPIOMode_TypeDef mode);
void Ds18b20_Reset(ds18b20_TypeDef ds);
void Ds18b20_WriteByte(ds18b20_TypeDef ds, int data);

int Ds18b20_ReadByte(ds18b20_TypeDef ds);
uint8_t Ds18b20_CRC(uint8_t* data, uint8_t bytes);

#define DS18B20_MODE_READ GPIO_Mode_IN_FLOATING
#define DS18B20_MODE_WRITE GPIO_Mode_Out_PP

#ifdef DS18B20_ADDR
void Ds18b20_Init(ds18b20_TypeDef *ds,GPIO_TypeDef* GPIO,uint32_t GPIO_Pin,uint8_t *addr){
	(*ds).addr=addr;
#else
void Ds18b20_Init(ds18b20_TypeDef *ds,GPIO_TypeDef* GPIO,uint32_t GPIO_Pin){
#endif
	GPIO_InitTypeDef GPIO_conf;
	(*ds).GPIO=GPIO;
	(*ds).flag=0;
	
	GPIO_conf.GPIO_Pin = GPIO_Pin;
  	GPIO_conf.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_conf.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIO, &GPIO_conf);
	(*ds).GPIO_conf=GPIO_conf;
	}
void Ds18b20_Mode(ds18b20_TypeDef ds,GPIOMode_TypeDef mode){
	ds.GPIO_conf.GPIO_Mode=mode;
	GPIO_Init(ds.GPIO, &ds.GPIO_conf);
	}

	
void Ds18b20_Reset(ds18b20_TypeDef ds){
	uint16_t timer;

	Ds18b20_Mode(ds,DS18B20_MODE_READ);
	while(!(ds.GPIO->IDR & ds.GPIO_conf.GPIO_Pin));
	__disable_irq();
	Ds18b20_Mode(ds,DS18B20_MODE_WRITE);
	ds.GPIO->ODR&=~ds.GPIO_conf.GPIO_Pin;
	__enable_irq();
	Delay_us(480);
	__disable_irq();
	Ds18b20_Mode(ds,DS18B20_MODE_READ);
	__enable_irq();
	Delay_us(410);
	}

void Ds18b20_WriteByte(ds18b20_TypeDef ds, int data){
	int i;
	for(i=0;i<8;i++){
		__disable_irq();
		Ds18b20_Mode(ds,DS18B20_MODE_WRITE);
		ds.GPIO->ODR&=~ds.GPIO_conf.GPIO_Pin;
		if(data & (1<<i)){
			Delay_us(10);
			ds.GPIO->ODR|=ds.GPIO_conf.GPIO_Pin;
			__enable_irq();
			Delay_us(55);
			}
		else{
			Delay_us(65);
			ds.GPIO->ODR|=ds.GPIO_conf.GPIO_Pin;
			__enable_irq();
			Delay_us(5);
			}
		}
	}
		
int Ds18b20_ReadByte(ds18b20_TypeDef ds){
	uint16_t i,x=0;

	for(i=0;i<8;i++){
		__disable_irq();
		Ds18b20_Mode(ds,DS18B20_MODE_WRITE);
		ds.GPIO->ODR&=~ds.GPIO_conf.GPIO_Pin;
		Delay_us(3);
		Ds18b20_Mode(ds,DS18B20_MODE_READ);
		Delay_us(10);
		if((ds.GPIO->IDR & ds.GPIO_conf.GPIO_Pin))
			x|=(1<<(i));
		__enable_irq();
		Delay_us(53);
		}
	return x;
	}
		
void Ds18b20_ConvertTemperature(ds18b20_TypeDef ds)
{
int i;

Ds18b20_Reset(ds); // Perform Master Reset of OneWire Bus
#ifdef DS18B20_ADDR
Ds18b20_WriteByte(ds, 0x55);
for(i=0;i<8;i++) Ds18b20_WriteByte(ds, ds.addr[i]);
#else
Ds18b20_WriteByte(ds, 0xCC);	 // skip ROM
#endif
Ds18b20_WriteByte(ds, 0x44);	 // convert temperature
//Ds18b20_Reset(ds);	 // Perform Master Reset of OneWire Bus
}

void Ds18b20_ReadTemperature(ds18b20_TypeDef *ds){
	uint8_t data[9];
	uint8_t ls,ms,ms1;
	uint16_t temp;
	int sig=0;
	int i;
	Ds18b20_ConvertTemperature(*ds);
	Ds18b20_Reset(*ds);
	#ifdef DS18B20_ADDR
	Ds18b20_WriteByte(ds, 0x55);
	for(i=0;i<8;i++) Ds18b20_WriteByte(ds, ds.addr[i]);
	#else
	Ds18b20_WriteByte(*ds, 0xCC);	 // skip ROM
	#endif
	Ds18b20_WriteByte(*ds, 0xBE);	 // read scratch pad
	memset(data,0,sizeof(data));
	for(i=0;i<9;i++)
		data[i]=Ds18b20_ReadByte(*ds);
#ifdef DS18B20_DEBUG
	printf("data: ");
	for(i=0;i<9;i++)
		printf("%02X ",data[i]);
	printf("crc=%02X \r\n",Ds18b20_CRC(data,9));
#endif
	if(!Ds18b20_CRC(data,9)){
		ms=data[1];
		ls=data[0];
		if(ms & 0xF0){
			ms=~(ms);
			ls=~ls+1;
			sig=1;
			}
		temp= ls | ms<<8;
		if(temp!=0xFF){
			if(!sig) (*ds).temp=(float)temp/16.0;
			else  (*ds).temp=(float)temp/-16.0;
			}
		}

}
void Ds18b20_ReadROM(ds18b20_TypeDef *ds){
	int i;

	Ds18b20_Reset(*ds);
	Ds18b20_WriteByte(*ds, 0x33);
	for(i=0;i<8;i++){
		//ds->addr[i]=Ds18b20_ReadByte(*ds);
		printf("0x%X ",Ds18b20_ReadByte(*ds));
	}

}

//calc_CRC - INTERNAL FUNCTION
//Purpose:    To calculate an 8-bit CRC based on a polynomial and the series
//            of data bytes
//Note:       Polynomial used x^8 + x^5 + x^4 + 1 = 10001100
//Inputs:     A pointer to an array of the data bytes and an int saying how many
//            bytes there are in the data array
//Outputs:    An int8 which is the calculated CRC
uint8_t Ds18b20_CRC(uint8_t* data, uint8_t bytes)
{
   #define CRC_POLY      0x8C
   uint8_t shift_register = 0, i, datab, bits;

   for(i = 0; i < bytes; ++i)
   {
      datab = *(data + i);
      for(bits = 0; bits < 8; ++bits)
      {
         if((shift_register ^ datab) & 0x01)
         {
            shift_register = shift_register >> 1;
            shift_register ^= CRC_POLY;
         }
         else
            shift_register = shift_register >> 1;
         datab = datab >> 1;
      }
   }
   return shift_register;
} //calc_CRC


