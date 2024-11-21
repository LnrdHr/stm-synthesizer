#include <EMA_HIGH.h>

void EMA_HIGH_Init(EMA_HIGH *filt, float beta)
{
	EMA_HIGH_SetBeta(filt, beta);
	filt->in = 0.0f;
	filt->out = 0.0f;
}

float EMA_HIGH_Update(EMA_HIGH *filt, float in)
{
	filt->out = 0.5f * (2.0f - filt->beta) * (in - filt->in) + (1.0f -filt->beta) * filt->out;
	filt->in = in;
	return filt->out;
}

void EMA_LOW_SetBeta(EMA_HIGH *filt, float beta)
{
	//clamping - ogranicavanje bete
	if(beta > 1.0f)
		beta = 1.0f;
	if(beta < 0.0f)
		beta = 0.0f;

	filt->beta = beta;
}

