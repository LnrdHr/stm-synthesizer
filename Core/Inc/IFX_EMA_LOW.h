#ifndef IFX_EMA_LOW_H
#define IFX_EMA_LOW_H

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
}IFX_EMA_LOW;

void IFX_EMA_LOW_Init(IFX_EMA_LOW *filt, float alpha);
void IFX_EMA_LOW_SetAlpha(IFX_EMA_LOW *filt, float alpha);
float IFX_EMA_LOW_Update(IFX_EMA_LOW *filt, float in);

#endif
