//#ifndef _Buzzer_H_
//#define _Buzzer_H_
//
//#include <stdio.h>
//#include <stdbool.h>
//void testMelody(){}
//
////#include "stm32f10x.h"
////#include "stm32f10x_rcc.h"
////#include "stm32f10x_gpio.h"
////#include "stm32f10x_tim.h"
////
////#define SYSCLK 72000000
////#define PRESCALER 72
////
////#define C	261	//Do
////#define C_	277 //Do#
////#define D	293 //Re
////#define D_	311 //Re#
////#define E	239 //Mi
////#define F	349 //Fa
////#define F_	370 //Fa#
////#define G 	392 //Sol
////#define G_	415 //Sol#
////#define A	440 //La
////#define A_	466 //La#
////#define H	494 //Si
////
////#define t1		2000
////#define t2		1000
////#define t4		500
////#define t8		250
////#define t16		125
////
////typedef struct
////{
////	uint16_t freq;
////	uint16_t time;
////}SoundTypeDef;
////
////#define MUSICSIZE 48
////
////const SoundTypeDef Music[MUSICSIZE] ={
////	{C*2, t4},
////	{G, t4},
////	{A_, t8},
////	{F, t8},
////	{D_, t8},
////	{F, t8},
////	{G, t4},
////	{C, t2},
////	{C*2, t4},
////	{G, t4},
////	{A_, t8},
////	{F, t8},
////	{D_, t8},
////	{F, t8},
////	{G, t4},
////	{C*2, t4},
////	{0, t8},
////	{D_, t8},
////	{D_, t8},
////	{D_, t8},
////	{G, t8},
////	{A_, t4},
////	{D_*2, t8},
////	{C_*2, t8},
////	{C*2, t8},
////	{C*2, t8},
////	{C*2, t8},
////	{C*2, t8},
////	{A_, t8},
////	{F, t8},
////	{D_, t8},
////	{F, t8},
////	{G, t4},
////	{C*2, t2},
////	{C*2, t2},
////	{A_, t8},
////	{G_, t8},
////	{G, t8},
////	{G_, t8},
////	{A_, t2},
////	{A_, t4},
////	{C*2, t4},
////	{A_, t8},
////	{F, t8},
////	{D_, t8},
////	{F, t8},
////	{G, t4},
////	{C*2, t2}
////};
////
////int MusicStep = 0;
////char PlayMusic = 0;
////
////void StartMusic(void) {
////	MusicStep = 0;
////	PlayMusic = 1;
////	sound(Music[MusicStep].freq, Music[MusicStep].time);
////}
////
////GPIO_InitTypeDef port;
////TIM_TimeBaseInitTypeDef timer;
////TIM_OCInitTypeDef timerPWM;
////
////int sound_time;
////int sound_counter;
////
////void sound_init(void) {
////	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
////
////	GPIO_StructInit(&port);
////	port.GPIO_Mode = GPIO_Mode_AF_PP;
////	port.GPIO_Pin = GPIO_Pin_6;
////	port.GPIO_Speed = GPIO_Speed_2MHz;
////	GPIO_Init(GPIOB, &port);
////
////	TIM_TimeBaseStructInit(&timer);
////	timer.TIM_Prescaler = PRESCALER;
////	timer.TIM_Period = 0xFFFF;
////	timer.TIM_ClockDivision = 0;
////	timer.TIM_CounterMode = TIM_CounterMode_Up;
////	TIM_TimeBaseInit(TIM2, &timer);
////
////	TIM_OCStructInit(&timerPWM);
////	timerPWM.TIM_Pulse = 0;
////	timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
////	timerPWM.TIM_OutputState = TIM_OutputState_Enable;
////	timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
////	TIM_OC1Init(TIM2, &timerPWM);
////
////    // Enable Interrupt by overflow
////    TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
////
////	//TIM_Cmd(TIM2, ENABLE);
////
////    // Enable Interrupt of Timer TIM2
////    NVIC_EnableIRQ(TIM2_IRQn);
////}
////
////void TIM2_IRQHandler(void){
////
////	if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)
////	  {
////	    // Reset flag
////	    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
////
////	    sound_counter++;
////	    if (sound_counter > sound_time) {
////	    	if (PlayMusic == 0) {
////	    		TIM_Cmd(TIM2, DISABLE);
////	    	}
////	    	else {
////	    		if (MusicStep < MUSICSIZE-1) {
////	    			if (TIM2->CCR1 == 0){
////	    				MusicStep++;
////	    				sound(Music[MusicStep].freq, Music[MusicStep].time);
////	    			}
////	    			else{
////	    				sound(0, 30);
////	    			}
////	    		}
////	    		else {
////		    		PlayMusic = 0;
////		    		TIM_Cmd(TIM2, DISABLE);
////	    		}
////	    	}
////	    }
////
////	    // over-capture
////	    if (TIM_GetFlagStatus(TIM2, TIM_FLAG_CC4OF) != RESET)
////	    {
////	      TIM_ClearFlag(TIM2, TIM_FLAG_CC4OF);
////	      // ...
////	    }
////	  }
////}
////
////void sound (int freq, int time_ms) {
////	if (freq > 0) {
////		TIM2->ARR = SYSCLK / timer.TIM_Prescaler / freq;
////		TIM2->CCR1 = TIM2->ARR / 2;
////	}
////	else {
////		TIM2->ARR = 1000;
////		TIM2->CCR1 = 0;
////	}
////	TIM_SetCounter(TIM2, 0);
////
////	sound_time = ((SYSCLK / timer.TIM_Prescaler / TIM2->ARR) * time_ms ) / 1000;
////	sound_counter = 0;
////	TIM_Cmd(TIM2, ENABLE);
////}
////
////int main(void)
////{
////	sound_init();
////
////	//sound (440, 1000);
////	StartMusic();
////
////	while(1)
////    {
////
////    }
////
////}
//#endif
