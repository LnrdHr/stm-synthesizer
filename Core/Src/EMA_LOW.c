#include <Ema_low.h>
#include <stdint.h>

float alpha = 0;
void EMA_LOW_Init(EMA_LOW *filt, float alpha)
{
	filt->out = 0.0f;
}

float EMA_LOW_Update(EMA_LOW *filt, uint16_t in)
{
	filt->out = alpha * in + (1-alpha) * filt->out;
	return filt->out;
}

