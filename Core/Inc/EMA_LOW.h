#ifndef EMA_LOW_H
#define EMA_LOW_H

typedef struct
{
	// EMA filter: diskretan, niskopropusni IIR(beskonacan impulsni odziv) filter prvog reda


	/*
	 * -alpha je broj između 0 i 1 i koristi se pri izračunu odziva sustava
	 * -kako se mijenja alpha, tako i cutoff frekvencija
	 * -alpha ~0 znaci da se djelovanje filtera nece povecavati ili smanjivati, dok alpha ~1 znaci da ce filter
	 * u potpunosti propustati ulazni signal
	 *
	 */
	float alpha;
	float out;
}EMA_LOW;

void EMA_LOW_Init(EMA_LOW *filt, float alpha);
void EMA_LOW_SetAlpha(EMA_LOW *filt, float alpha);
float EMA_LOW_Update(EMA_LOW *filt, float in);

#endif
