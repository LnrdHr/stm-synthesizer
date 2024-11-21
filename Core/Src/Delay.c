#include <Delay.h>
#include "math.h"

void delay_Init(Delay *delay, float mix, float feedback, float delayTime_ms, float sampleRate_Hz)
{
	delay_SetLength(delay, delayTime_ms, sampleRate_Hz);

	delay->mix = mix;
	delay->feedback = feedback;

	delay->lineIndex = 0;

	for(int i=0; i< DELAY_LINE_MAX_LENGTH; ++i)
		delay->line[i] = 0.0f;

	delay->out = 0.0f;
}

float delay_Update(Delay *delay, float inputSample)
{
	float delayLineOutput = delay->line[delay->lineIndex];
	float delayLineInput = inputSample + delay->feedback * delayLineOutput;

	delay->line[delay->lineIndex] = delayLineInput;

	delay->lineIndex++;
	if(delay->lineIndex >= delay->lineLength)
		delay->lineIndex = 0;

	delay->out = (1.0f - delay->mix) * inputSample + delay->mix * delayLineOutput;

	if(delay->out > 1.0f)
		delay->out = 1.0f;
	else if(delay->out < -1.0f)
		delay->out = -1.0f;

	return delay->out;
}

void delay_SetLength(Delay *delay, float delayTime_ms, float sampleRate_Hz)
{
	delay->lineLength = round(0.001f * delayTime_ms * sampleRate_Hz);

	if(delay->lineLength > DELAY_LINE_MAX_LENGTH)
		delay->lineLength = DELAY_LINE_MAX_LENGTH;
}
