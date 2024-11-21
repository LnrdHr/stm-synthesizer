#include "ADSR.h"
#include <stdint.h>
#include "math.h"

ADSR adsr;
int counter=0; //brojac koji provjerava da li smo dosli do kraja pojedinog segmenta
float Nsamples=0; //broj uzoraka segmenta
uint16_t b1=0; //pocetna tocka segmenta
uint16_t b2=0; //krajnja tocka segmenta
uint16_t out=0; //amplitudni izlaz

//inicijalizacija ovojnice
void ADSR_Init(int samplingRate, float aTime, float dTime, float sLevel, float rTime)
{
	adsr.state=offState;
	adsr.attackTime=aTime;
	adsr.decayTime=dTime;
	adsr.sustainLevel=sLevel;
	adsr.releaseTime=rTime;
	adsr.samplingRate=samplingRate;
	adsr.triggered = 0;
	adsr.released = 0;
}


/*provjera u kojem je stanju ovojnica
 *za svaki segment ovojnice se racuna trajanje u obliku broja uzoraka(Nsamples)
 *
 *
*/
float ADSR_Update(int in)
{
	switch (adsr.state)
	{
	case offState:
		if(adsr.triggered==1)
		{
			adsr.state =attackState;
			counter=0;
			Nsamples = adsr.samplingRate * adsr.attackTime * 1000; // mnozenje s 1000 zbog izrazavanja vremena pojedinog segmenta u ms
			b1=0;
			b2=1;
		}
		out = 0;
		break;

	case attackState:
		if(counter==Nsamples)
		{
			adsr.state=decayState;
			counter = 0;
			Nsamples = adsr.samplingRate * adsr.decayTime * 1000;
			b1=b2;
			b2=adsr.sustainLevel;
		}
		out = ADSR_CalculateNextSample(Nsamples, in, b1, b2);
		break;

	case decayState:
		if(counter==Nsamples)
		{
			adsr.state=sustainState;
			counter=0;
			b1=b2=adsr.sustainLevel;
		}
		out = ADSR_CalculateNextSample(Nsamples, in, b1, b2);
		break;

	case sustainState:
		if(adsr.released==1)
		{
			adsr.state=releaseState;
			counter=0;
			Nsamples = adsr.samplingRate * adsr.releaseTime * 1000;
			b1=adsr.sustainLevel;
			b2=0;
		}
		out = ADSR_CalculateNextSample(Nsamples, in, b1, b2);
		break;

	case releaseState:
		//ovojnica ne mora biti u off stanju da bi se mogla ponovno pokrenuti
		if(adsr.triggered==1)
		{
			adsr.state=attackState;
			counter=0;
			Nsamples = adsr.samplingRate * adsr.attackTime * 1000;
			b1=0;
			b2=1;
			out=0;
			break;
		}
		if(counter==Nsamples)
		{
			adsr.state=offState;
			counter=0;
		}
		out = ADSR_CalculateNextSample(Nsamples, in, b1, b2);
		break;
	}
	return out;
}

//izlaz = ulaz + (Krajnja tocka segmenta - Prva tocka segmenta) / Broj uzoraka segmenta
uint16_t ADSR_CalculateNextSample(float Nsamples, uint16_t in, float b1, float b2)
{
	out = round(in + (b2-b1) / Nsamples);
	counter++;
	return out;
}

void ADSR_Trigger()
{
	adsr.triggered = 1;
}

void ADSR_Release()
{
	adsr.released = 1;
	adsr.triggered = 0;
}

void ADSR_Reset()
{
	adsr.released = 0;
	adsr.triggered = 0;
}
