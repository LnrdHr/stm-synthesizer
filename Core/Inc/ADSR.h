#ifndef ADSR_H
#define ADSR_H

#include <stdint.h>

//adsr ovojnica
typedef struct
{
	int samplingRate;
	//attackState=1, decayState=2,...
	enum State
	{
		offState= 0,
		attackState,
		decayState,
		sustainState,
		releaseState
	}state;

	float attackTime;
	float decayTime;
	float sustainLevel;
	float releaseTime;
	uint32_t counter;
	uint32_t Nsamples; //broj uzoraka segmenta
	float b1; //pocetna tocka level segmenta
	float b2; //krajnja tocka level segmenta

	float out;  //izlazna vrijednost amplitude nakon djelovanja ovojnice
	char triggered; //mijenja  je trigger() funkcija u funkciji HAL_UART_RxCpltCallback() u main.c
	char released; //mijenja je release() funkcija u funkciji HAL_UART_RxCpltCallback() u main.c
	char finished;
}Adsr;

uint16_t ADSR_Update(Adsr adsr, unsigned int in);

#endif
