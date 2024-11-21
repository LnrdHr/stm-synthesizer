#ifndef EMA_HIGH_H
#define EMA_HIGH_H

typedef struct
{
	// EMA filter: diskretan, niskopropusni IIR(beskonacan impulsni odziv) filter
	float in;
	float beta;
	float out;

}EMA_HIGH;

void EMA_HIGH_Init(EMA_HIGH *filt, float beta);
void EMA_HIGH_SetBeta(EMA_HIGH *filt, float beta);
float EMA_HIGH_Update(EMA_HIGH *filt, float in);

#endif
