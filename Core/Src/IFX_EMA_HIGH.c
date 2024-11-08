#include <IFX_EMA_HIGH.h>

void IFX_EMA_HIGH_Init(IFX_EMA_HIGH *filt, float beta)
{
	IFX_EMA_HIGH_SetBeta(filt, beta);
	filt->in = 0.0f;
	filt->out = 0.0f;
}

float IFX_EMA_HIGH_Update(IFX_EMA_HIGH *filt, float in)
{
	filt->out = 0.5f * (2.0f - filt->beta) * (in - filt->in) + (1.0f -filt->beta) * filt->out;
	filt->in = in;
	return filt->out;
}

void IFX_EMA_LOW_SetBeta(IFX_EMA_HIGH *filt, float beta)
{
	//clamping - ogranicavanje alphe
	if(beta > 1.0f)
		beta = 1.0f;
	if(beta < 0.0f)
		beta = 0.0f;

	filt->beta = beta;
}

