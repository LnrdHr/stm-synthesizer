#ifndef ADSR_H
#define ADSR_H

#include <stdint.h>

//adsr ovojnica
typedef struct
{
	int samplingRate;

	enum State
	{
		//attackState=1, decayState=2,...
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

	enum State state; //trenutno stanje ovojnice
	float out;  //izlazna vrijednost amplitude nakon djelovanja ovojnice
	char triggered; //mijenja  je trigger() funkcija u funkciji HAL_UART_RxCpltCallback() u main.c
	char released; //mijenja je release() funkcija u funkciji HAL_UART_RxCpltCallback() u main.c
}ADSR;

ADSR* ADSR_Init(int samplingRate, float attackTime, float decayTime, float sustainLevel, float releaseTime);

//state machine funkcija koja se nalazi u callbacku ArangeSamples
uint16_t ADSR_Update(ADSR* adsr, unsigned int in);

/*

//postavlja triggered zastavicu u 1
void ADSR_Trigger();
//postavlja released zastavicu u 1, a triggered u 0(da ne bi slucajno bili u stanju released i triggered istovremeno)
void ADSR_Release();
//resetira obje zastavice u 0
void ADSR_Reset();
*/

#endif
