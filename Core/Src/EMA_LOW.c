#include <EMA_LOW.h>

void EMA_LOW_Init(EMA_LOW *filt, float alpha)
{
	EMA_LOW_SetAlpha(filt, alpha);
	filt->out = 0.0f;
}

float EMA_LOW_Update(EMA_LOW *filt, float in)
{
	filt->out = filt->alpha * in + (1-filt->alpha) * filt->out;
	return filt->out;
}

void EMA_LOW_SetAlpha(EMA_LOW *filt, float alpha)
{
	//clamping - ogranicavanje alphe
	if(alpha > 1.0f)
		alpha = 1.0f;
	if(alpha < 0.0f)
		alpha = 0.0f;

	filt->alpha = alpha;
}

