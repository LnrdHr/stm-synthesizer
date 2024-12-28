#include "ADSR.h"
#include "Voice.h"
#include "math.h"

Voice Voice_Init(ADSR ovojnica, char nota, char notaVelo)
{
	Voice voice;
	voice.nota = nota;
	voice.notaVelo = notaVelo;
	voice.frekvencija_f = 440.0f * (pow(2,((voice.nota - 69) * 0.0833333f ))) / 2;
	voice.notaStanje = 1;
	voice.pomakUTablici_f = 2048.0f * voice.frekvencija_f / 44000.0f;
	voice.accFaze_f = 0;
	voice.adsr = ovojnica;
	voice.adsr.triggered = 1;
	voice.aktivan = 1;
	return voice;
}
