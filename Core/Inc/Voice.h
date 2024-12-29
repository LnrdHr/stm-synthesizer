#ifndef VOICE_H
#define VOICE_H
#include "ADSR.h"

struct Voice
{
	int notaStanje;
	char nota;
	char notaVelo;
	float frekvencija_f;
	float pomakUTablici_f;
	float accFaze_f;
	struct ADSR adsr;
	int aktivan;
};

struct Voice Voice_Init(struct ADSR adsr, char nota, char notaVelo);


#endif
