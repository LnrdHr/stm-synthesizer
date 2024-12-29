#ifndef ADSR_H
#define ADSR_H

#include <stdint.h>

//adsr ovojnica
struct ADSR
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
	uint16_t counter;
};

struct ADSR ADSR_Init(float attackTime, float decayTime, float sustainLevel, float releaseTime);

//state machine funkcija koja se nalazi u callbacku ArangeSamples
uint16_t ADSR_Update(struct ADSR adsr, unsigned int in);

#endif
