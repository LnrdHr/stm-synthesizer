#include "ADSR.h"
#include <stdint.h>
#include "math.h"

 uint32_t counter=0; //brojac koji provjerava da li smo dosli do kraja pojedinog segmenta
 uint16_t Nsamples=0; //broj uzoraka segmenta
 float b1=0; //pocetna tocka level segmenta
 float b2=0; //krajnja tocka level segmenta

//inicijalizacija ovojnice
ADSR* ADSR_Init(int samplingRate, float aTime, float dTime, float sLevel, float rTime)
{
 ADSR* adsr;
 adsr->state=offState;
 adsr->attackTime=aTime;
 adsr->decayTime=dTime;
 adsr->sustainLevel=sLevel;
 adsr->releaseTime=rTime;
 adsr->samplingRate=samplingRate;
 adsr->triggered = 0;
 adsr->released = 0;
 return adsr;
}

/*provjera u kojem je stanju ovojnica
 *za svaki segment ovojnice se racuna trajanje u obliku broja uzoraka(Nsamples)
 *
 *
*/

uint16_t ADSR_Update(ADSR* adsr, unsigned int in)
{
	uint16_t out=0; //amplitudni izlaz
 switch (adsr->state)
 {
 case offState:
  if(adsr->triggered==1)
  {
   adsr->state =attackState;
   counter=0;
   Nsamples = (adsr->samplingRate * adsr->attackTime)  ;
   b1=0;
   b2=1;

  }
  out = 0;
  break;

 case attackState:
  if(counter==Nsamples)
  {
   adsr->state=decayState;
   counter = 0;
   Nsamples = (adsr->samplingRate * adsr->decayTime)  ;
   b1=b2;
   b2=adsr->sustainLevel;
   break;
  }

  out = (int)(counter * in * (b2-b1) / Nsamples);
  counter++;

  break;

 case decayState:
  if(counter==Nsamples)
  {
   adsr->state=sustainState;
   counter=0;
   b1=b2=adsr->sustainLevel;
  }
  out =  (in * b1) -  (int)(counter * in * (b1- b2) / Nsamples);
  counter++;

  break;

 case sustainState:
  if(adsr->released==1)
  {
   adsr->state=releaseState;
   counter=0;
   Nsamples = (adsr->samplingRate * adsr->releaseTime) ;
   b1=adsr->sustainLevel;
   b2=0;
   break;
  }
  out = round(in * b2);
   counter++;

  break;

 case releaseState:

  if(adsr->triggered==1)
  {
   adsr->state=attackState;
   counter=0;
   Nsamples = (adsr->samplingRate * adsr->attackTime);
   b1=0;
   b2=1;
   out=0;
   break;
  }
  if(counter==Nsamples)
  {
   adsr->state=offState;
   counter=0;
   break;
  }
  out =  (in * b1) - round(counter * in * (b1- b2) / Nsamples);
  counter++;

  break;
 }
 return out;
}

/*
void ADSR_Trigger()
{
 adsr->triggered = 1;
}

void ADSR_Release()
{

 adsr->released = 1;
 adsr->triggered = 0;
}

void ADSR_Reset()
{

 adsr->released = 0;
 adsr->triggered = 0;
}
*/

