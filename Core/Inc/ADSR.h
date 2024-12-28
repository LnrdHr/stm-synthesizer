#ifndef ADSR_H
#define ADSR_H

#include <stdint.h>

//adsr ovojnica
typedef struct
{

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

	float out;  //izlazna vrijednost amplitude nakon djelovanja ovojnice
	char triggered;
	char released;
}ADSR;

ADSR ADSR_Init(int samplingRate, float attackTime, float decayTime, float sustainLevel, float releaseTime);

//state machine funkcija koja se nalazi u callbacku ArangeSamples
uint16_t ADSR_Update(ADSR adsr, unsigned int in);

#endif
