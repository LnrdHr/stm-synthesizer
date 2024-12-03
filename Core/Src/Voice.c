#include "ADSR.h"
#include "Voice.h"
#include "math.h"

void Voice_Init(Voice* voice, ADSR* ovojnica, char nota, char notaVelo)
{
	voice->nota = nota;
	voice->notaVelo = notaVelo;
	voice->frekvencija_f = 440.0f * (pow(2,((voice->nota - 69) * 0.0833333f ))) / 2;
	voice->notaStanje = 1;
	voice->pomakRadnogPolja_f = 2048.0f * voice->frekvencija_f / 44000.0f;
	voice->accFaze_f = 0;
	voice->ovojnica = ovojnica;
	voice->ovojnica->triggered = 1;
}
