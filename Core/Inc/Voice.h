#ifndef VOICE_H
#define VOICE_H

#include "ADSR.h"

typedef struct
{
	int notaStanje;
	char nota;
	char notaVelo;
	float frekvencija_f;
	float pomakRadnogPolja_f;
	float accFaze_f;
	ADSR* ovojnica;
	int aktivan;
}Voice;

Voice* Voice_Init(ADSR* adsr, char nota, char notaVelo);

#endif
