#ifndef IFX_EMA_HIGH_H
#define IFX_EMA_HIGH_H

typedef struct
{
	// EMA filter: diskretan, niskopropusni IIR(beskonacan impulsni odziv) filter
	float in;
	float beta;
	float out;

}IFX_EMA_HIGH;

void IFX_EMA_HIGH_Init(IFX_EMA_HIGH *filt, float beta);
void IFX_EMA_HIGH_SetBeta(IFX_EMA_HIGH *filt, float beta);
float IFX_EMA_HIGH_Update(IFX_EMA_HIGH *filt, float in);

#endif
