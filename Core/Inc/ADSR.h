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
	};

	float attackTime;
	float decayTime;
	float sustainLevel;
	float releaseTime;


	enum State state; //trenutno stanje ovojnice
	float out;  //izlazna vrijednost amplitude nakon djelovanja ovojnice
	int triggered;
	int released;

}ADSR;

void ADSR_Init(ADSR* adsr, int samplingRate, float attackTime, float decayTime, float sustainLevel, float releaseTime);

//izracun sljedeceg uzorka na temelju stanja ovojnice
uint16_t ADSR_CalculateNextSample(float Nsamples, uint16_t input, float b1, float b2);

//state machine funkcija koja se nalazi u callbackovima ArangeSamplesInHalfBuff i ArangeSamplesInFullBuff
float ADSR_Update(ADSR* adsr, unsigned int in);

/*
//postavlja triggered zastavicu u 1
void ADSR_Trigger();
//postavlja released zastavicu u 1, a triggered u 0(da ne bi slucajno bili u stanju released i triggered istovremeno)
void ADSR_Release();
//resetira obje zastavice u 0
void ADSR_Reset();
*/

#endif
