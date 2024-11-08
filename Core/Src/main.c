/* USER CODE BEGIN Header */
/**
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tabele.h"
#include "math.h"
#include <IFX_EMA_LOW.h>
#include <IFX_EMA_HIGH.h>
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

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
#define FULL_BUFFER_SIZE 2048
#define HALF_BUFFER_SIZE 1024
#define ADC_BUFFER_SIZE 2
#define SAMPLING_FREQ 44000
 uint16_t dma_in[FULL_BUFFER_SIZE];
 uint16_t dma_out[FULL_BUFFER_SIZE];
 uint16_t adc1_buf[ADC_BUFFER_SIZE]; // buffer za adc potenciometar
 uint8_t rxBuff[3];  //Buffer za MIDI RX
 uint8_t stanjeTipki[16]; // Koje su tipke pritisnute
 uint8_t glasnocaTonova[16];
 uint8_t brojAktiviranihTipki =0;
 uint16_t triangleTable[FULL_BUFFER_SIZE];
 int32_t i;
 // uint16_t ad_rez = 0;
 // uint16_t adc_index =0;
 uint32_t sumAdc=0;
 uint32_t pointerRadnogPolja = 0;
 uint16_t numADCconvert = 0;
 //float outIndex =0;
 float ADCGain =0;  // treba biti izmedju 0 i 1 obavezno!!
 uint16_t WorkingBuffer[FULL_BUFFER_SIZE]= {0};
 float f; //frekvencija note
 float readPointer_f;
 uint32_t readPointer_i;
 float pomakRadnogPolja_f;
 IFX_EMA_LOW low_filt;
 IFX_EMA_HIGH high_filt;
 float ALPHA = 0.0f;
 float BETA = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//DMA za citanje analognih ulaza - potenciometri
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
{
 uint16_t ad_rez = 0;
				  	sumAdc = sumAdc + adc1_buf[0];
				  	numADCconvert ++;
				  	if (numADCconvert >= 250)
				  	{
				  		ad_rez = sumAdc / 250;
				  		ad_rez = ad_rez >> 2;
				  		numADCconvert = 0;
				  		sumAdc = 0;
				  		ADCGain = ad_rez /1024.0;
				  	}
}
////////////////////////////////////////////////////
//DMA slanja prve polovice buffera na DAC
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef* hi2s2)
{
   ArangeSamplesInHalfBuff();

	for (int i =0; i < HALF_BUFFER_SIZE; i++)
	  {
		  {
			  dma_out[i] =  (0.1 * ADCGain *(WorkingBuffer[i]))  ;
		  }
	  }
}
////////////////////////////////////////////////////
//DMA slanja druge polovice buffera na DAC
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef* hi2s2)
{
 	 ArangeSamplesInFullBuff();

	for (int i = HALF_BUFFER_SIZE; i < FULL_BUFFER_SIZE; i++)
	  {
		  {
			  dma_out[i] =  (0.1 * ADCGain *(WorkingBuffer[i]))  ;
		  }
	  }

}
////////////////////////////////////////////////////
// MIDI IN obrada
void  HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart1)
{
   char notaStanje = rxBuff[0];  // ON ili OFF
   char nota = rxBuff[1];  // koja tipka u MIDI sustavu
   char notaVelo = rxBuff[2];   // glasnoca

   if (notaStanje == 0x90)  // nota ON
   {
	//   for (int i =0; i <16; i++)
	   i =0;
	   {
		   if (stanjeTipki[i]== 0 )
		   {
			   stanjeTipki[i] = nota;
			   glasnocaTonova[i] = notaVelo;
			   brojAktiviranihTipki++;
			   //Izracun frekv iz MIDI note
			    f= 440.0 * (pow(2,((nota - 69) * 0.0833333 )));
			    // Izracun  pomaka pokazivaca u tablici
			    // ReadPointer = vel. tablice * f /sampling frekv.
			    pomakRadnogPolja_f = 2048.0 * f / 44000.0;
			   // readPointer_f =  f;
			     readPointer_f = 0;
			 //  break;

		   }

	   }
   }

   if (notaStanje == 0x80)  // nota OFF
    {
  	   for (int i =0; i <16; i++)
  	   {
  		   if (stanjeTipki[i]== nota )
  		   {
  			   stanjeTipki[i] = 0;
  			   glasnocaTonova[i] = 0;
  			   brojAktiviranihTipki--;
  			   break;
  		   }
  	   }
     }

}
////////////////////////////////////////////////////
// Priprema prve polovice buffera dok traje DMA druge polovice
void  ArangeSamplesInHalfBuff(void)
{
   //
	pointerRadnogPolja = 0;
	    while(pointerRadnogPolja < HALF_BUFFER_SIZE)  //priprema radnog buffera
	    {  //Ogranicavanje rubnog uvjeta
	    	while (readPointer_f >= FULL_BUFFER_SIZE)
	    	{
	    		readPointer_f = readPointer_f - FULL_BUFFER_SIZE ;
	    		readPointer_i = round(readPointer_f);
	    	}

	    	//low pass filter processing
	    	/*
	    	float in_f = (float)sineLUT[readPointer_i]; //convert to float
	    	float inProcessed_f = IFX_EMA_LOW_Update(&low_filt,in_f);
	    	uint16_t inProcessed_i = round(inProcessed_f); //convert back to int
			*/

	    	//high pass filter processing

	    	float in_f = (float)sineLUT[readPointer_i]; //convert to float
			float inProcessed_f = IFX_EMA_HIGH_Update(&high_filt,in_f);
			uint16_t inProcessed_i = round(inProcessed_f); //convert back to int


	    	WorkingBuffer[pointerRadnogPolja] = inProcessed_i;


	    	if (pointerRadnogPolja < FULL_BUFFER_SIZE)
	    		pointerRadnogPolja++;
	    	readPointer_f =  readPointer_f + pomakRadnogPolja_f;
	    	readPointer_i = round(readPointer_f);
	    }
}
////////////////////////////////////////////////////
// Priprema druge polovice buffera dok traje DMA prve polovice
void  ArangeSamplesInFullBuff(void)
{
   //
	    while (pointerRadnogPolja < FULL_BUFFER_SIZE)  //priprema radnog buffera
	    {  //Ogranicavanje rubnog uvjeta
	    	while (readPointer_f >= (FULL_BUFFER_SIZE))
	    	{
	    		readPointer_f = readPointer_f - FULL_BUFFER_SIZE ;
	    		readPointer_i = round(readPointer_f);
	    	}

	    	//low pass filter processing
			/*
			float in_f = (float)sineLUT[readPointer_i]; //convert to float
			float inProcessed_f = IFX_EMA_LOW_Update(&low_filt,in_f);
			uint16_t inProcessed_i = round(inProcessed_f); //convert back to int
			*/

			//high pass filter processing
			float in_f = (float)sineLUT[readPointer_i]; //convert to float
			float inProcessed_f = IFX_EMA_HIGH_Update(&high_filt,in_f);
			uint16_t inProcessed_i = round(inProcessed_f); //convert back to int

			WorkingBuffer[pointerRadnogPolja] = inProcessed_i;

	    	if (pointerRadnogPolja < FULL_BUFFER_SIZE)
	    		pointerRadnogPolja++;
	    	readPointer_f = readPointer_f + pomakRadnogPolja_f;
	    	readPointer_i = round(readPointer_f);
	    }
}
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
  IFX_EMA_LOW_Init(&low_filt, ALPHA);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*) dma_out, FULL_BUFFER_SIZE);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_buf, ADC_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart1, rxBuff, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
  /* User can add his own implementation to report the HAL error return state */
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
