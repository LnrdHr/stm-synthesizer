#include <IFX_EMA_LOW.h>

void IFX_EMA_LOW_Init(IFX_EMA_LOW *filt, float alpha)
{
	IFX_EMA_LOW_SetAlpha(filt, alpha);
	filt->out = 0.0f;
}

float IFX_EMA_LOW_Update(IFX_EMA_LOW *filt, float in)
{
	filt->out = filt->alpha * in + (1-filt->alpha) * filt->out;
	return filt->out;
}

void IFX_EMA_LOW_SetAlpha(IFX_EMA_LOW *filt, float alpha)
{
	//clamping - ogranicavanje alphe
	if(alpha > 1.0f)
		alpha = 1.0f;
	if(alpha < 0.0f)
		alpha = 0.0f;

	filt->alpha = alpha;
}

