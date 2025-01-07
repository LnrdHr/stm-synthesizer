/* USER CODE BEGIN Header */
/** verzija polifonije s ovojnicom. Treba srediti mjesanje attacka i releasea.
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
// Radi ADSR i monofonija.Dodat i UART2
// Polifonija s zajednickom ovojnicom
// Skraceno vrijeme pripreme buffera, jer se salje isti sample na L i R kanal.
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tabele.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include <Delay.h>
#include <Ema_low.h>
#include "Adsr.h"
#include "Voice.h"
#include "../../ECUAL/I2C_LCD/I2C_LCD.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
#define ENVELOPE
#define TABLE_SIZE 2048
#define FULL_BUFFER_SIZE 2048
#define HALF_BUFFER_SIZE 1024
#define QUARTER_BUFFER_SIZE 512
#define ADC_BUFFER_SIZE 6
#define SAMPLING_FREQ 44000
#define TABLE_1 sawLUT
#define TABLE_2 sineLUT
#define POLYNUM 6
#define MyI2C_LCD I2C_LCD_1

int16_t dma_out[FULL_BUFFER_SIZE];
uint16_t adc1_buf[ADC_BUFFER_SIZE]; // buffer za adc potenciometar
uint8_t rxBuff[3];  //Buffer za MIDI RX
uint8_t stanjeTipki[8]; // Koje su tipke pritisnute
uint8_t glasnocaTonova[8];
uint8_t brojAktiviranihTipki =0;
uint32_t sumAdc=0;
uint32_t indeksRadnogPolja_uw = 0;
uint16_t numADCconvert = 0;
float ADCGain=0;  // treba biti izmedju 0 i 1 obavezno!!
float prevADCGain = 0;
int ADCGain_changed_flag = 0;
uint32_t pot2 = 0;
uint32_t pot3 = 0;
float pot2_f = 0;
uint16_t WorkingBuffer[HALF_BUFFER_SIZE]= {0};
float f; //frekvencija note
float accFaze_f;
float pomakRadnogPolja_f;
uint16_t inProcessed_ui;
EMA_LOW lp_filter;
extern float alpha;
float attack_f = 0.2;
float decay_f =0.2;
float sustain_f= 0.9f;
float release_f=0.4f;
uint32_t  sumSample_uw = 0;
char str[12];
struct Voice voices[6] ={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void  ArangeSamplesInBuff(int dioBuffera);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//DMA za citanje analognih ulaza - potenciometri
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
	uint16_t ad_rez = 0, ad_rez3 = 0;
	sumAdc = sumAdc + adc1_buf[0];
	pot2 = adc1_buf[1];
	pot3 += adc1_buf[2];
	numADCconvert ++;
	if (numADCconvert >= 250)
	{
		ad_rez = sumAdc / 250;
		ad_rez3 = pot3 / 250;
		numADCconvert = 0;
		sumAdc = 0;
		pot3 = 0;
		ADCGain = ad_rez /1024.0f;
		if(ADCGain > 0.5) ADCGain = 0.5;
		if(ADCGain != prevADCGain){
			ADCGain_changed_flag = 1;
		}

		pot2_f = pot2 / 1024.f;
		alpha = ad_rez3 / 1024.f;
		if(alpha == 0) alpha = 0.004f;
	}
}
////////////////////////////////////////////////////
//DMA slanja prve polovice buffera na DAC
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef* hi2s2)
{
	int32_t temp_w;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	ArangeSamplesInBuff(QUARTER_BUFFER_SIZE);

	uint16_t j=0;
	for (int i =0; i < QUARTER_BUFFER_SIZE; i++)
	{
		temp_w  = (WorkingBuffer[i]);
		// temp_w = 32768 - temp_w ;	// centriranje signala oko nule

		dma_out[j] = (ADCGain * temp_w);

		dma_out[j+1] =  dma_out[j]  ;
		j= j+2;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

////////////////////////////////////////////////////
//DMA slanja druge polovice buffera na DAC
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef* hi2s2)
{
	int32_t temp_w;
	uint16_t j=0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	ArangeSamplesInBuff(HALF_BUFFER_SIZE);
	j = HALF_BUFFER_SIZE;
	for (int i = QUARTER_BUFFER_SIZE; i < HALF_BUFFER_SIZE; i++)
	{
		temp_w  = (WorkingBuffer[i]);
		// temp_w = 32768 - temp_w ; // centriranje signala oko nule
		dma_out[j] = (ADCGain * temp_w) ;
		dma_out[j+1] =  dma_out[j];
		j= j+2;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}
////////////////////////////////////////////////////
// MIDI IN obrada
void  HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart1)
{
#define NOTA_ON 1
#define NOTA_NUM 2
#define NOTA_VELO 3
	char notaStanje = rxBuff[0];  // ON ili OFF
	char nota = rxBuff[1];  // koja tipka u MIDI sustavu
	char notaVelo = rxBuff[2];  // velocity odnosno glasnoca tona

	//////////////
	if (notaStanje == 0x90)  // nota ON
	{
		for (int i =0; i <POLYNUM; i++)
		{

			if (stanjeTipki[i]== 0 ) // ako mjesto u polju nije zauzeto
			{
				stanjeTipki[i] = nota;
				voices[i].nota = nota;
				glasnocaTonova[i] = notaVelo;
				voices[i].notaVelo = notaVelo;
				brojAktiviranihTipki++;

				//Izracun frekv iz MIDI note
				voices[i].frekvencija = (440.0f * (pow(2,((voices[i].nota - 69) * 0.0833333f )))) / 2;
				// Izracun  pomaka pokazivaca u tablici
				// ReadPointer = vel. tablice * f /sampling frekv.
				voices[i].pomakRadnogPolja_f = TABLE_SIZE * voices[i].frekvencija / SAMPLING_FREQ;
				if ( voices[i].Ovojnica.finished)
				{
					voices[i].accFaze_f = 0;
				}
				voices[i].Ovojnica.triggered = 1;
				voices[i].Ovojnica.released = 0;
				break;
			}
		}
	}  // end if nota == ON

	else if (notaStanje == 0x80)  // nota OFF
	{
		for (int i =0; i <POLYNUM; i++)
		{
			if (stanjeTipki[i]== nota )
			{
				if ( voices[i].Ovojnica.finished)
				{
					stanjeTipki[i] = 0;
					glasnocaTonova[i] = 0;
					brojAktiviranihTipki--;
				}
				{
					voices[i].Ovojnica.triggered = 0;
					voices[i].Ovojnica.released = 1;
				}
			}
		}
	}
}
////////////////////////////////////////////////////

void  ArangeSamplesInBuff(int dioBuffera)
{
	uint32_t accFaze_uw;
	sumSample_uw =0; //sadrzi ukupnu vrijednost svih sampla koja se dijeli s brojem aktiviranih tipki
	if (dioBuffera == QUARTER_BUFFER_SIZE)
	{
		indeksRadnogPolja_uw = 0;
	}
	while(indeksRadnogPolja_uw < dioBuffera)  //priprema radnog buffera HALF_BUF ili FULL_BUF
	{  //Ogranicavanje rubnog uvjeta
		for (int i=0; i < POLYNUM; i++)  // izracun jednog sample koji je zbroj svih
		{

			while (voices[i].accFaze_f  + voices[i].pomakRadnogPolja_f >= TABLE_SIZE)
			{
				voices[i].accFaze_f = voices[i].accFaze_f - TABLE_SIZE ;
				accFaze_uw = (int) voices[i].accFaze_f;
			}
			voices[i].accFaze_f =  voices[i].accFaze_f + voices[i].pomakRadnogPolja_f;
			accFaze_uw = round(voices[i].accFaze_f) ;
			uint32_t sample_ui =  ((TABLE_1[accFaze_uw]) * pot2_f + TABLE_2[accFaze_uw]) / 2;

			/* ---------------------------------------------------------*/
			switch (voices[i].Ovojnica.state)
			{
			case offState:
				//	printf("%d\n", voices[i].Ovojnica.state
				if(voices[i].Ovojnica.triggered==1)
				{
					voices[i].Ovojnica.finished = 0;
					voices[i].Ovojnica.released = 0;
					voices[i].Ovojnica.state = attackState;
					voices[i].Ovojnica.Nsamples = (SAMPLING_FREQ * voices[i].Ovojnica.attackTime)  ;//
					voices[i].Ovojnica.b1=0;
					voices[i].Ovojnica.b2=1;
				}
				voices[i].Ovojnica.out = 0;
				break;

			case attackState:
				if(voices[i].Ovojnica.counter == voices[i].Ovojnica.Nsamples)
				{
					voices[i].Ovojnica.out = (int)(voices[i].Ovojnica.counter * sample_ui * (voices[i].Ovojnica.b2-voices[i].Ovojnica.b1) / voices[i].Ovojnica.Nsamples);// za vrijeme < 1 s
					voices[i].Ovojnica.state=decayState;
					voices[i].Ovojnica.counter = 0;
					voices[i].Ovojnica.Nsamples = (SAMPLING_FREQ * voices[i].Ovojnica.decayTime);
					voices[i].Ovojnica.b1=voices[i].Ovojnica.b2;
					voices[i].Ovojnica.b2=voices[i].Ovojnica.sustainLevel;
					break;
				}

				voices[i].Ovojnica.out = (int)(voices[i].Ovojnica.counter * sample_ui * (voices[i].Ovojnica.b2-voices[i].Ovojnica.b1) / voices[i].Ovojnica.Nsamples);// za vrijeme < 1 s
				voices[i].Ovojnica.counter++;

				break;

			case decayState:
				if(voices[i].Ovojnica.counter==voices[i].Ovojnica.Nsamples)
				{
					voices[i].Ovojnica.state=sustainState;
					voices[i].Ovojnica.counter=0;
					voices[i].Ovojnica.b1=voices[i].Ovojnica.b2=voices[i].Ovojnica.sustainLevel;
				}
				voices[i].Ovojnica.counter++;
				voices[i].Ovojnica.out =  (sample_ui * voices[i].Ovojnica.b1) -  (int)(voices[i].Ovojnica.counter * sample_ui * (voices[i].Ovojnica.b1- voices[i].Ovojnica.b2) / voices[i].Ovojnica.Nsamples);

				break;

			case sustainState:
				if(voices[i].Ovojnica.released ==1)
				{
					voices[i].Ovojnica.out = round(sample_ui * voices[i].Ovojnica.b2);
					voices[i].Ovojnica.state=releaseState;
					voices[i].Ovojnica.counter=0;
					voices[i].Ovojnica.Nsamples = (SAMPLING_FREQ * voices[i].Ovojnica.releaseTime) ;
					voices[i].Ovojnica.b1=voices[i].Ovojnica.sustainLevel;
					voices[i].Ovojnica.b2=0;
					break;
				}
				voices[i].Ovojnica.out = round(sample_ui * voices[i].Ovojnica.b2);
				break;

			case releaseState:
				if(voices[i].Ovojnica.counter==voices[i].Ovojnica.Nsamples)
				{
					voices[i].Ovojnica.state = offState;
					voices[i].Ovojnica.counter = 0;
					voices[i].Ovojnica.finished = 1;

					stanjeTipki[i] = 0;
					glasnocaTonova[i] = 0;
					brojAktiviranihTipki--;

					break;
				}
				voices[i].Ovojnica.out =  (sample_ui * voices[i].Ovojnica.b1) - (int)(voices[i].Ovojnica.counter * sample_ui * (voices[i].Ovojnica.b1- voices[i].Ovojnica.b2) / voices[i].Ovojnica.Nsamples);
				voices[i].Ovojnica.counter++;
				break;
			}  // end switch
			/* --------------------------------------------------------------*/
			inProcessed_ui = voices[i].Ovojnica.out;

			sumSample_uw = sumSample_uw + inProcessed_ui;
		}  //end for

		sumSample_uw = sumSample_uw  / POLYNUM;
		sumSample_uw = round(EMA_LOW_Update(&lp_filter, sumSample_uw));
		WorkingBuffer[indeksRadnogPolja_uw] = sumSample_uw ;
		sumSample_uw =0;
		if (indeksRadnogPolja_uw < HALF_BUFFER_SIZE)
			indeksRadnogPolja_uw++;
	}  //end while
}
////////////////////// write funkcija  za printf
int _write(int fd, char* ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
	return len;
}
//////////////////////
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	EMA_LOW_Init(&lp_filter, alpha);

	for (int i = 0; i < POLYNUM; i++)
	{
		voices[i].Ovojnica.attackTime = attack_f;
		voices[i].Ovojnica.decayTime = decay_f;
		voices[i].Ovojnica.sustainLevel = sustain_f;
		voices[i].Ovojnica.releaseTime = release_f;
		voices[i].Ovojnica.state = offState;
		voices[i].Ovojnica.finished = 1;
	}
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */


	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2S2_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) dma_out, FULL_BUFFER_SIZE);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_buf, ADC_BUFFER_SIZE);
	HAL_UART_Receive_DMA(&huart1, rxBuff, 3);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);  //LED za debug

	I2C_LCD_Init(MyI2C_LCD);
	I2C_LCD_SetCursor(MyI2C_LCD, 0, 0);
	I2C_LCD_WriteString(MyI2C_LCD, "Lero");
	I2C_LCD_SetCursor(MyI2C_LCD, 0, 1);
	I2C_LCD_WriteString(MyI2C_LCD, "synth");
	HAL_Delay(2000);
	I2C_LCD_Clear(MyI2C_LCD);
	HAL_Delay(500);
	I2C_LCD_WriteString(MyI2C_LCD, "Volume: ");
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1)
	{
		HAL_Delay(500);
		if(ADCGain_changed_flag){
			I2C_LCD_SetCursor(MyI2C_LCD, 7, 0);
			int a = ADCGain * 100 * 2;
			itoa(a, str, 10); //integer to ascii
			I2C_LCD_WriteString(MyI2C_LCD, "   ");
			I2C_LCD_SetCursor(MyI2C_LCD, 7, 0);
			I2C_LCD_WriteString(MyI2C_LCD, str);
			ADCGain_changed_flag = 0;
		}
		prevADCGain = ADCGain;

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2S2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S2_Init(void)
{

	/* USER CODE BEGIN I2S2_Init 0 */

	/* USER CODE END I2S2_Init 0 */

	/* USER CODE BEGIN I2S2_Init 1 */

	/* USER CODE END I2S2_Init 1 */
	hi2s2.Instance = SPI2;
	hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
	hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
	hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_44K;
	hi2s2.Init.CPOL = I2S_CPOL_LOW;
	hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
	if (HAL_I2S_Init(&hi2s2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2S2_Init 2 */

	/* USER CODE END I2S2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 31250;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 2000000;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return Ovojnica */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
