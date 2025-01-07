#include "Adsr.h"
#include <stdint.h>
#include "math.h"
#include "stdio.h"

//uint32_t counter=0; //brojac koji provjerava da li smo dosli do kraja pojedinog segmenta

/*
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
	adsr.finished = 0;
	adsr.counter = 0;
}
*/

/*provjera u kojem je stanju ovojnica
 *za svaki segment ovojnice se racuna trajanje u obliku broja uzoraka(Nsamples)
 *
 *
*/
uint16_t ADSR_Update(Adsr adsr, unsigned int in)
{
	uint32_t Nsamples=0; //broj uzoraka segmenta
	float b1=0; //pocetna tocka level segmenta
	float b2=0; //krajnja tocka level segmenta
	uint16_t out=0; //amplitudni izlaz
	if(adsr.state != offState)	printf("%d\n", adsr.state);
	switch (adsr.state)
	{
	case offState:
	//	printf("%d\n", adsr.state
		if(adsr.triggered==1)
		{
			adsr.finished = 0;
			adsr.state =attackState;
			adsr.counter=0;
			Nsamples = (adsr.samplingRate * adsr.attackTime)  ;//
			b1=0;
			b2=1;
		}
		out = 0;
		break;

	case attackState:
		if(adsr.counter==Nsamples)
		{
			adsr.state=decayState;
			adsr.counter = 0;
			Nsamples = (adsr.samplingRate * adsr.decayTime);
			b1=b2;
			b2=adsr.sustainLevel;
			break;
		}
     //    temp =(b2-b1) / Nsamples * in;  //Ako zelimo attack time > 1s tada koristimo double
      //    out = (int)(counter * temp);
 		out = (int)(adsr.counter * in * (b2-b1) / Nsamples);// za vrijeme < 1 s
		adsr.counter++;

		break;

	case decayState:
		if(adsr.counter==Nsamples)
		{
			adsr.state=sustainState;
			adsr.counter=0;
			b1=b2=adsr.sustainLevel;
		}
		adsr.counter++;
		out =  (in * b1) -  (int)(adsr.counter * in * (b1- b2) / Nsamples);
		break;

	case sustainState:
		if(adsr.released ==1)
		{
			adsr.state=releaseState;
			adsr.counter=0;
			Nsamples = (adsr.samplingRate * adsr.releaseTime) ;
			b1=adsr.sustainLevel;
			b2=0;
			break;
		}
		out = round(in * b2);
		break;

	case releaseState:
		//ovojnica ne mora biti u off stanju da bi se mogla ponovno pokrenuti
/*		if(adsr.triggered==1)
		{
			adsr.state=attackState;
			adsr.counter=0;
			Nsamples = (adsr.samplingRate * adsr.attackTime);
			b1=0;
			b2=1;
			out=0;
			break;
		}
*/
		if(adsr.counter==Nsamples)
		{
			adsr.state=offState;
			adsr.counter=0;
			adsr.finished = 1;
			break;
		}
		out =  (in * b1) - (int)(adsr.counter * in * (b1- b2) / Nsamples);
		adsr.counter++;
		//printf("%d\n", );
		break;
	}
	return out;
}

