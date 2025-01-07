#ifndef VOICE_H
#define VOICE_H
#include <Adsr.h>
struct Voice{
	 char nota;
	 char notaVelo;
	 float frekvencija;
	 float accFaze_f;
	 float pomakRadnogPolja_f;
	 uint16_t sampleVal_ui;
	 Adsr Ovojnica;
 };


#endif
