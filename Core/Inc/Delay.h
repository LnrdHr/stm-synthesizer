#define DELAY_LINE_MAX_LENGTH 44000 //max number of samples that can be delayed
#include <stdint.h>

typedef struct
{
	//delay time = line length / sample rate
	float mix; //[0,1] 1->wet(only delayed signal), 0->dry(only input signal)
	float feedback; //number of (decaying in volume) repeats

	float line[DELAY_LINE_MAX_LENGTH];

	uint32_t lineIndex;
	uint32_t lineLength;

	float out;
}Delay;

void delay_Init(Delay *delay, float mix, float feedback, float delayTime_ms, float sampleRate_Hz);
float delay_Update(Delay *delay, float inputSample);
void delay_SetLength(Delay *delay, float delayTime_ms, float sampleRate_Hz);
