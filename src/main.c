#include "dht.h"
#include "stm32f10x.h"
#include "conf.h"
//#include "controle.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ds18b20.h"
#include "xbee.h"
#include "media.h"
#include "arctl.h"
#include "fila.h"
#include "pid.h"

//#define DEBUG_BUF

#define SYS_GPIO	GPIOB
#define SYS_0		GPIO_Pin_6
#define SYS_1		GPIO_Pin_5
#define SYS_2		GPIO_Pin_4
#define SYS_3		GPIO_Pin_3

#define RELE1_ON	GPIOB->ODR|=GPIO_Pin_15
#define RELE1_OFF	GPIOB->ODR&=~GPIO_Pin_15

#define RELE2_ON	GPIOB->ODR|=GPIO_Pin_15
#define RELE2_OFF	GPIOB->ODR&=~GPIO_Pin_15

#define RELE3_ON	GPIOB->ODR|=GPIO_Pin_15
#define RELE3_OFF	GPIOB->ODR&=~GPIO_Pin_15

uint8_t xbee_broadcast[]={0,0,0,0,0,0,0xFF,0xFF};

ds18b20_TypeDef dsInt,dsExt,dsDis;
dht_TypeDef dht;
void func_process();
void xbee_process();
void disp_init();
void pisca_led();
void gpio_init();
void sys_cod();
void disp_cod();

#define LED_OK	GPIO_Pin_9

xbee_t xbee;
EE_t eep;
TTFila fila;
PID_T pidTemp;
floatbyte_T floataux;
uint8_t payload[100],conf;
EXTI_InitTypeDef EXTI_InitStructure;
int i,j;
uint32_t start_pisca=0;
Media_T tempIntm,tempExtm,tempDism;

uint8_t flag_rect=0;
uint32_t time_pisca,time_temp,time_send,time_ping;
int main(){
	int cnt_temp=0;
	float tempInt,tempExt,tempDis;
	//__disable_irq();
	//configura perifericos
	Delay_Init();
	TIM1_PWM();
	fila_FVazia(&fila);
	usart2_init();

	usart1_init();
	//__enable_irq();
	printf("Iniciando...\r\n");
	//configura o GPIO
	gpio_init();
	//calcula o codigo do sistema
	sys_cod();
	//calcula o codigo do dispositivo
	disp_cod();
	//atualizando o id do sistema
	printf("Start do dispositivo.. %d\r\n",SystemCoreClock);

	Delay_ms(1000);
	xbee.start=2;


	printf("Atualizando o id do sistema..,\r\n");
	payload[0]='I';
	payload[1]='D';
	payload[2]=0x05;
	payload[3]=0x55;
	//payload[4]=eep.disp.id_sys;
	xbee_cmdAT(&xbee,payload,4);
	//inicia os sensores
	Ds18b20_Init(&dsExt,GPIOB,GPIO_Pin_1);
	Ds18b20_Init(&dsDis,GPIOB,GPIO_Pin_10);
	Ds18b20_Init(&dsInt,GPIOB,GPIO_Pin_11);
	//dht_init(&dht,GPIOC,GPIO_Pin_15);
	Ds18b20_ReadTemperature(&dsInt);
	Ds18b20_ReadTemperature(&dsExt);
	Ds18b20_ReadTemperature(&dsDis);
	//Ds18b20_ReadTemperature(&ds1);
	//dht_read(&dht);
	tempExt=dsExt.temp;
	tempDis=dsDis.temp;
	tempInt=dsInt.temp;
	//inicio do dispositivo

	media_Init(&tempExtm,5);
	media_Init(&tempDism,5);
	media_Init(&tempIntm,5);

	//eep.disp.tempo.var=5;
	//EE_Write(eep);
	//
	EE_Read(&eep);
	/*eep.disp.setpoint.val=2.0;
	eep.disp.p.val=10;
	eep.disp.i.val=5;
	eep.disp.d.val=2;
	EE_Write(eep);*/

	GPIOB->ODR|=LED_OK;
	eep.disp.status=0;
	//uint8_t temp,humid;
	switch(eep.disp.tipo){
		case 5:
			pid_init(&pidTemp,eep.disp.p.val,eep.disp.i.val,eep.disp.d.val,0,1,1);
			break;
		case 6:
			pid_init(&pidTemp,eep.disp.p.val,eep.disp.i.val,eep.disp.d.val,0,1,0);
			break;
		}
	TIM1->CCR1=(uint16_t)((SystemCoreClock / 50000 ) - 1);
	Delay_ms(2000);
	time_pisca=millis();
	time_temp=millis();
	while(1){
		pisca_led();
		xbee_process();

		if(timeout(time_temp)>=1000){
			//printf("ds\n\r");
			Ds18b20_ReadTemperature(&dsInt);
			Ds18b20_ReadTemperature(&dsExt);
			Ds18b20_ReadTemperature(&dsDis);
			//printf("temp: Int:%2.2f Dis:%2.2f Ext:%2.2f \n\r",dsInt.temp,dsDis.temp,dsExt.temp);

			//if(!(dht.humid>100 || dht.humid<0 || dht.temp>100 || dht.temp<-15 || ds.temp>100 || ds.temp<-15 || ds1.temp>100 || ds1.temp<-15 || ds.temp==85 || ds1.temp==85)){
			//	if(!((ds.temp>=(temp1+10)) | (ds1.temp>=(temp2+10)) | (dht.temp>=(temp3+10)) | (dht.humid>=(humid+10)))){
					//printf("  temp1=%2.2f temp2=%2.2f temp3=%2.2f humid=%2.2f \r\n",ds.temp,ds1.temp,dht.temp,dht.humid);
					media_process(&tempIntm,dsInt.temp);
					media_process(&tempExtm,dsExt.temp);
					media_process(&tempDism,dsDis.temp);
					printf("temp: Int:%2.2f Dis:%2.2f Ext:%2.2f \n\r",tempIntm.media,tempDism.media,tempExtm.media);
					//if(temp1!=ds.temp){

						switch(eep.disp.tipo){
							case 5:
							case 6:
								pid_process(&pidTemp,(eep.disp.setpoint.val)/50.0,(tempIntm.media/50.0));
								eep.disp.pot=(pidTemp.out);
								break;
							case 3:
								//aquecimento
								if(tempIntm.media>(eep.disp.setpoint.val+eep.disp.hist.val))
									//desliga
									eep.disp.pot=0;

								else if(tempIntm.media<(eep.disp.setpoint.val-eep.disp.hist.val))
									//liga
									eep.disp.pot=1;

								break;
							case 4:
								//resfriamento
								if(tempIntm.media>(eep.disp.setpoint.val+eep.disp.hist.val))
									//liga
									eep.disp.pot=1;
								else if(tempIntm.media<(eep.disp.setpoint.val-eep.disp.hist.val))
									//desliga
									eep.disp.pot=0;

								break;
							}

						TIM1->CCR1=(uint16_t)((SystemCoreClock / 25000 ) - 1)*(1-eep.disp.pot);
						printf("p=%3.2f\n\r",eep.disp.pot);
						//}

					//printf("in=%1.4f out=%1.4f\r\n",(temp1m.media/100.0),pidTemp.out);


					//controle();
					tempInt=dsInt.temp;
					tempExt=dsExt.temp;
					tempDis=dsDis.temp;
					if(cnt_temp<10) cnt_temp++;
				//	}
				//}
			time_temp=millis();
			}
		if(eep.disp.status==3){
			if((timeout(time_send) >=((eep.disp.tempo.var)*1000)) & cnt_temp>=10){
				//printf("send\n\r");
				memset(payload,0,sizeof(payload));
				__disable_irq();
				payload[0]=ARCTL_TEMP;
				payload[1]=eep.disp.id_disp;
				payload[2]=5;
				//temp 1
				payload[3]=1;
				floataux.val=tempIntm.media;
				for(i=0;i<4;i++)
					payload[4+i]=floataux.byte.b[i];
				//temp2
				payload[8]=2;
				floataux.val=tempExtm.media;
				for(i=0;i<4;i++)
					payload[9+i]=floataux.byte.b[i];
				//temp3
				payload[13]=3;
				floataux.val=tempDism.media;
				for(i=0;i<4;i++)
					payload[14+i]=floataux.byte.b[i];
				//umid
				payload[18]=4;
				floataux.val=eep.disp.pot*10;
				for(i=0;i<4;i++)
					payload[19+i]=floataux.byte.b[i];
				payload[23]=5;
				floataux.val=eep.disp.setpoint.val;
				for(i=0;i<4;i++)
					payload[24+i]=floataux.byte.b[i];
				__enable_irq();
				xbee_SendData(&xbee,xbee.addr_cord,payload,28);
				//printf("m temp1=%2.2f temp2=%2.2f temp3=%2.2f humid=%2.2f \r\n",temp1m.media,temp2m.media,temp3m.media,humidm.media);
				time_send=millis();
				}
			}
		if(timeout(time_ping)>3000 & eep.disp.status>=2){
			payload[0]=ARCTL_PING1;
			xbee_SendData(&xbee,xbee.addr_cord,payload,1);
			if(flag_rect>253) flag_rect=4;
			else flag_rect++;
			//printf("status=%d\r\n",eep.disp.status);
			time_ping=millis();
			}

		}
	}

void USART2_IRQHandler(void){
	if(USART_GetFlagStatus(USART2,USART_IT_RXNE)){
	xbee_usart(&fila,USART_ReceiveData(USART2));
	}
	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

void xbee_process(void){
	TipoFilaDado dado;
	uint8_t qtd,tipo;
	if(fila.tam>0){
		fila_remove(&dado,&fila);
		memcpy(xbee.buf,dado.dado,dado.tam);
		xbee_reciver(&xbee);
#ifdef DEBUG_BUF
		//if(xbee.buf[3]!=0x8b){
			printf("%d rec: ",eep.disp.status);
			for(i=0;i<xbee.buf[2]+4;i++)
				printf("%02X ",xbee.buf[i]);
			printf("\r\n");
		//	}
#endif
		switch(xbee.type){
			case XBEE_STATUS:
				if(xbee.buf[4]==2){
					disp_init();
					//printf("send1\r\n");

				}
			break;
			//recebe resposta de comando AT
			case XBEE_CMDAT:
				//se for o comando para atualizar o id do sistema e o status for o inicial
				if(xbee.buf[5]=='I' & xbee.buf[6]=='D' & xbee.buf[7]==0 & eep.disp.status==0){
					//tenta iniciar o dispositiv no mestre
					printf("ID\r\n");
					xbee_cmdAT(&xbee,"OP",2);
				}
				if(xbee.buf[5]=='O' & xbee.buf[6]=='P' & xbee.buf[7]==0 & eep.disp.status==0){
					printf("OP\r\n");
					if(xbee.buf[14]==0x05 & xbee.buf[15]==0x55){
						disp_init();
					}
					else{
						Delay_ms(2000);
						xbee_cmdAT(&xbee,"OP",2);
					}
				}
				break;
			//recebe resposta de transmissão
			case XBEE_TRANSMIT_STATUS:
				//verifica se foi sucesso
				if(xbee.buf[8]==0 & flag_rect>2){
					GPIOB->ODR|=LED_OK;
					eep.disp.status=2;
				}
				break;
			case XBEE_RECEIVE_PACKET:
				if(xbee.buf[XBEE_OFFSET_PAYLOAD]==ARCTL_MESTRE){
					for(i=0;i<8;i++)
						xbee.addr_cord[i]=xbee.buf[XBEE_OFFSET_SOURCE+i];
					disp_init();
				}
				if(xbee.buf[XBEE_OFFSET_PAYLOAD]==ARCTL_INIT & (eep.disp.status==1)){
					//GPIOB->ODR&=~LED_OK;
					//printf("teste\n\r");
					eep.disp.status=2;
				}
				else{

				}
				//se o dispositivo tiver inicializado, inicia os demais processos
				//if(eep.disp.status>=3){
					switch(xbee.buf[XBEE_OFFSET_PAYLOAD]){
						case ARCTL_TEMP:
							//printf("temp volta\r\n");
							break;
						case ARCTL_SET_TEMP:
							conf=xbee.buf[XBEE_OFFSET_PAYLOAD+1];
							tipo=xbee.buf[XBEE_OFFSET_PAYLOAD+2];
							eep.disp.tipo=tipo;
							for(j=0;j<4;j++){
								eep.disp.setpoint.byte.b[j]=xbee.buf[XBEE_OFFSET_PAYLOAD+3+j];
								eep.disp.hist.byte.b[j]=xbee.buf[XBEE_OFFSET_PAYLOAD+7+j];
								eep.disp.p.byte.b[j]=xbee.buf[XBEE_OFFSET_PAYLOAD+11+j];
								eep.disp.i.byte.b[j]=xbee.buf[XBEE_OFFSET_PAYLOAD+15+j];
								eep.disp.d.byte.b[j]=xbee.buf[XBEE_OFFSET_PAYLOAD+19+j];
								}
							eep.disp.tempo.byte.b[0]=xbee.buf[XBEE_OFFSET_PAYLOAD+23];
							eep.disp.tempo.byte.b[1]=xbee.buf[XBEE_OFFSET_PAYLOAD+24];
							printf("update set: %2.2f hist: %2.2f tempo: %d p=%2.2f i=%2.2f d=%2.2f modo=%d\r\n",	eep.disp.setpoint.val,
																											eep.disp.hist.val,
																											eep.disp.tempo.var,
																											eep.disp.p.val,
																											eep.disp.i.val,
																											eep.disp.d.val,
																											eep.disp.tipo);
							switch(eep.disp.tipo){
								case 5:
									//resfria
									pid_init(&pidTemp,eep.disp.p.val,eep.disp.i.val,eep.disp.d.val,0,1,1);
									break;
								case 6:
									//aquece
									pid_init(&pidTemp,eep.disp.p.val,eep.disp.i.val,eep.disp.d.val,0,1,0);
									break;
								}
							EE_Write(eep);
							break;
						case ARCTL_PING:
							flag_rect=0;
							//printf("ping\r\n");
							GPIOB->ODR&=~LED_OK;
							eep.disp.status=3;
							payload[0]=ARCTL_PING;
							xbee_SendData(&xbee,xbee.addr_cord,payload,1);
							break;
						case ARCTL_PING1:
							flag_rect=0;
							GPIOB->ODR&=~LED_OK;
							eep.disp.status=3;
							///printf("ping1\r\n");
							break;

						}

					//}
			break;
		}
		}
	}
void disp_init(){
	payload[0]=ARCTL_INIT;
	payload[1]=eep.disp.id_disp;
	xbee_SendData(&xbee,xbee_broadcast,payload,2);
	eep.disp.status=1;
}
void pisca_led(){
	if(timeout(time_pisca)>500){
		GPIOC->ODR&=~GPIO_Pin_13;
		Delay_ms(50);
		GPIOC->ODR|=GPIO_Pin_13;
		time_pisca=millis();
		}
}
void gpio_init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/*GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  SYS_0 | SYS_2 | SYS_2 | SYS_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SYS_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  0xFF; // (GPIO_Pin_0 ate o 7)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);*/
}
void sys_cod(){
	uint8_t aux=0;
	if(!(SYS_GPIO->IDR & SYS_0)) aux=1;
	if(!(SYS_GPIO->IDR & SYS_1)) aux|=2;
	if(!(SYS_GPIO->IDR & SYS_2)) aux|=4;
	if(!(SYS_GPIO->IDR & SYS_3)) aux|=8;
	printf("sys=%02X \n\r",aux);
	//eep.disp.id_sys=aux;
}
void disp_cod(){
	printf("disp=%02X \n\r",(~GPIOA->IDR & 0xFF));
	eep.disp.id_disp=3;//(~GPIOA->IDR & 0xFF);
}

