#include "stm32f10x_conf.h"
#include "conf.h"
#include <stdio.h>
#include "stm32f10x.h"

//#define ADC1_DR_Address    ((uint32_t)0x4001244C)

//__IO uint16_t ADCConvertedValue[10];

uint32_t us=0,ms=0;
NVIC_InitTypeDef NVIC_InitStructure;
void Delay_us(__IO uint32_t nTime)
{
	uint32_t start,end;
	start=TIM3->CNT;
  	while((uint16_t)(TIM3->CNT - start)<=nTime);

}
void Delay_ms(__IO uint32_t nTime)
{
uint16_t start;

	start=TIM2->CNT;
  	while((uint16_t)(TIM2->CNT - start)<=nTime);

}
void Delay_Init(){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1;
	TIM_TimeBaseStructure.TIM_Period = UINT16_MAX;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM3, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 100) - 1;
	TIM_TimeBaseStructure.TIM_Period = UINT16_MAX;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);

	//tim4
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/2000)-1;// 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_Period = 1;//120;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, ENABLE);

	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	//NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,0));

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	//TIM2->SR&=~TIM_IT_Update;
	//TIM2->DIER=TIM_DIER_UIE;
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM4_IRQn);

	//SysTick_Config (SystemCoreClock/1000000);

}
void TIM4_IRQHandler(void){
	if (TIM4->SR & TIM_SR_UIF){
		TIM4->SR &= ~TIM_SR_UIF;
		ms++;
		//GPIOC->ODR^=GPIO_Pin_13;
		}
	}
uint32_t millis(){
	return ms;
}
uint32_t timeout(uint32_t t){
	return (millis() - t);
}
void ADC1_Initm(void){
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);

	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);

	while(ADC_GetCalibrationStatus(ADC1));


	/*NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);*/

	ADC_RegularChannelConfig(ADC1, 5, 1, ADC_SampleTime_239Cycles5);

	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}
void TIM1_PWM(void){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0;//, Channel2Pulse = 0, Channel3Pulse = 0, Channel4Pulse = 0;
GPIO_InitTypeDef GPIO_InitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

/* GPIOA Configuration: Channel 1, 2 and 3 as alternate function push-pull */
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;// | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* GPIOB Configuration: Channel 1N, 2N and 3N as alternate function push-pull */
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  //GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* TIM1 Configuration ---------------------------------------------------
   Generate 7 PWM signals with 4 different duty cycles:
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
   and Connectivity line devices and to 24 MHz for Low-Density Value line and
   Medium-Density Value line devices

   The objective is to generate 7 PWM signal at 17.57 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   The channel 1 and channel 1N duty cycle is set to 50%
   The channel 2 and channel 2N duty cycle is set to 37.5%
   The channel 3 and channel 3N duty cycle is set to 25%
   The channel 4 duty cycle is set to 12.5%
   The Timer pulse is calculated as follows:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
  ----------------------------------------------------------------------- */
  /* Compute the value to be set in ARR regiter to generate signal frequency at 17.57 Khz */
  TimerPeriod = (SystemCoreClock / 25000 ) - 1;
  /* Compute CCR1 value to generate a duty cycle at 50% for channel 1 and 1N */
  Channel1Pulse = (uint16_t) (((uint32_t) 50 * (TimerPeriod - 1)) / 100);
  /* Compute CCR2 value to generate a duty cycle at 37.5%  for channel 2 and 2N */
  //Channel2Pulse = (uint16_t) (((uint32_t) 375 * (TimerPeriod - 1)) / 1000);
  /* Compute CCR3 value to generate a duty cycle at 25%  for channel 3 and 3N */
  //Channel3Pulse = (uint16_t) (((uint32_t) 25 * (TimerPeriod - 1)) / 100);
  /* Compute CCR4 value to generate a duty cycle at 12.5%  for channel 4 */
 // Channel4Pulse = (uint16_t) (((uint32_t) 125 * (TimerPeriod- 1)) / 1000);

  /* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = Channel1Pulse;
  //TIM_OC2Init(TIM1, &TIM_OCInitStructure);

  //TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
  //TIM_OC3Init(TIM1, &TIM_OCInitStructure);

  //TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
  //TIM_OC4Init(TIM1, &TIM_OCInitStructure);

  	/*NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
  	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
  	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);*/

  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);

  /* TIM1 Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void EXTI_config(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;// | GPIO_Pin_2 | GPIO_Pin_3 ;//| GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger	= EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line9;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*EXTI_InitStructure.EXTI_Line 	= EXTI_Line7;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line8;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line 	= EXTI_Line9;
	EXTI_Init(&EXTI_InitStructure);*/


	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;
	NVIC_Init(&NVIC_InitStructure);


}
void usart1_init(){

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Configure USART Tx as push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(USART1, ENABLE);
   /* NVIC_InitStructure.NVIC_IRQChannel = 37;//USART1_IRQn;
   	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   	 NVIC_Init(&NVIC_InitStructure);
   	 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);*/

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}
void usart2_init(){
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    /* Configure USART Tx as push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(USART2, &USART_InitStructure);

    //NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0,1));
    NVIC_InitStructure.NVIC_IRQChannel = 38;//USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    /* Enable USART */
    USART_Cmd(USART2, ENABLE);

    //setvbuf(stdin, NULL, _IONBF, 0);
    //setvbuf(stdout, NULL, _IONBF, 0);
    //setvbuf(stderr, NULL, _IONBF, 0);
}


void usart3_init(){
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

   /* // Configure USART Tx as push-pull
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Configure USART Rx as input floating
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);*/

    // Configure USART Tx as push-pull
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

   // Configure USART Rx as input floating
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
   GPIO_Init(GPIOB, &GPIO_InitStructure);

    //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

    /* USART configuration */
    USART_Init(USART3, &USART_InitStructure);

    /* Enable USART */

    //NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
   // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStructure);
    //USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART3, ENABLE);

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

void usart_send(USART_TypeDef *usart,char *str){
	while(*str){
		while ((usart->SR & USART_FLAG_TC) == (uint16_t) RESET);
		usart->DR = (*str++ & (uint16_t)0x01FF);

	}
}
void usart_send_n(USART_TypeDef *usart,char *str, int n){
	int i;
	for(i=0;i<n;i++){
		while ((usart->SR & USART_FLAG_TC) == (uint16_t) RESET);
		usart->DR = (str[i] & (uint16_t)0x01FF);
	}
}
