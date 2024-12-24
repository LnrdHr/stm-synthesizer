/* USER CODE BEGIN Header */
/** verzija pseudopolifonija s ovojnicom
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
#include <EMA_HIGH.h>
#include <EMA_LOW.h>
#include <Delay.h>
#include <ADSR.h>
#include <VOICE.h>


//#include <MIDI.h>
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
#define CURRENT_LUT triangleLUT
#define POLYNUM 6
 int16_t dma_out[FULL_BUFFER_SIZE];
 uint16_t adc1_buf[ADC_BUFFER_SIZE]; // buffer za adc potenciometar
 uint8_t rxBuff[3];  //Buffer za MIDI RX
 uint8_t stanjeTipki[16]; // Koje su tipke pritisnute
 uint8_t glasnocaTonova[16];
 uint8_t brojAktiviranihTipki =0;
 int32_t i;
 // uint16_t ad_rez = 0;
 // uint16_t adc_index =0;
 uint32_t sumAdc=0;
 uint32_t indeksRadnogPolja_uw = 0;
 uint16_t numADCconvert = 0;
 //float outIndex =0;
 float ADCGain =0;  // treba biti izmedju 0 i 1 obavezno!!
 uint16_t WorkingBuffer[HALF_BUFFER_SIZE]= {0};
 float f; //frekvencija note
 float accFaze_f;
 float pomakRadnogPolja_f;
 uint16_t inProcessed_ui;
// void  ArangeSamplesInFullBuff();
 //void  ArangeSamplesInHalfBuff();
 void  ArangeSamplesInBuff(int dioBuffera);


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
				  		ADCGain = ad_rez /1024.0f;
				  	}
}
////////////////////////////////////////////////////
//DMA slanja prve polovice buffera na DAC
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef* hi2s2)
{
	int32_t temp_w;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	 ArangeSamplesInBuff(QUARTER_BUFFER_SIZE);

//	ADCGain = brojAktiviranihTipki * 0.03f; // // pojacanje zbog ponistavanja radi razl. faza
   ADCGain = 0.2f;
   uint16_t j=0;
   for (int i =0; i < QUARTER_BUFFER_SIZE; i++)
	  {
	   	   	   temp_w  = (WorkingBuffer[i]);
	   	   	  // temp_w = 32768 - temp_w ;	// centriranje signala oko nule
	   	   	   dma_out[j] = (ADCGain * temp_w)    ;
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
// 	ADCGain = brojAktiviranihTipki * 0.03f; // pojacanje zbog ponistavanja radi razl. faza
 	ADCGain = 0.2f;
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

   char notaStanje = rxBuff[0];  // ON ili OFF
   char nota = rxBuff[1];  // koja tipka u MIDI sustavu
   char notaVelo = rxBuff[2];  // velocity odn. glasnoca tona
	#if defined  ENVELOPE
   	   	   ADSR_Reset();
	#endif
   	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
   if (notaStanje == 0x90)  // nota ON
   {
	  // AddMIDInote();
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
	   			    voices[i].accFaze_f = 0;  // 500 da pocne od nule i nema klika kod attacka
	   				#if defined  ENVELOPE
	   			    	voices[i].Ovojnica.triggered = 1;
	   			    	adsr.triggered = 1;
	   				#endif
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

  			   stanjeTipki[i] = 0;
  			   glasnocaTonova[i] = 0;
  			   brojAktiviranihTipki--;
  			//   f = 0;
				#if defined  ENVELOPE
  			   	   voices[i].Ovojnica.triggered = 0;
  			   	   voices[i].Ovojnica.released = 1;
  			    //   voices[i].pomakRadnogPolja_f = 0;
  			   //    voices[i].accFaze_f = 0;
  			   	  // ADSR_Release();
				#endif
  			   	adsr.released = 1;
  			   	adsr.triggered = 0;
  			   break;
  		   }
  	   }
     }
}
////////////////////////////////////////////////////

void  ArangeSamplesInBuff(int dioBuffera)
{
	 uint32_t accFaze_uw;
	 uint32_t  sumSample_uw =0; //sadrzi ukupnu vrijednost svih sampla koja se dijeli s brojem aktiviranih tipki

	 if (dioBuffera == QUARTER_BUFFER_SIZE)   // prvi  dio
	 {
		 indeksRadnogPolja_uw = 0;
	 }
			while(indeksRadnogPolja_uw < dioBuffera)  //priprema radnog buffera HALF_BUF ili FULL_BUF
			{  //Ogranicavanje rubnog uvjeta
				for (int i=0; i < POLYNUM; i++) 		// izracun 1 sample
				{

						while (voices[i].accFaze_f >= TABLE_SIZE)
						{
							voices[i].accFaze_f = voices[i].accFaze_f - TABLE_SIZE ;
							accFaze_uw = (int) voices[i].accFaze_f;
						}
						voices[i].accFaze_f =  voices[i].accFaze_f + voices[i].pomakRadnogPolja_f;
						accFaze_uw = round(voices[i].accFaze_f) ;
						sumSample_uw = (sumSample_uw + CURRENT_LUT[accFaze_uw]);

				 }  //end for

				sumSample_uw = sumSample_uw  / POLYNUM;
			//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
				inProcessed_ui = ADSR_Update(sumSample_uw);
			//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
				   WorkingBuffer[indeksRadnogPolja_uw] = inProcessed_ui;
				   sumSample_uw =0;
				if (indeksRadnogPolja_uw < HALF_BUFFER_SIZE)
					  indeksRadnogPolja_uw++;

	    	  }  //end while

		if (adsr.finished == 1)
		{
			for (int i=0; i < POLYNUM; i++)
			{
				voices[i].accFaze_f = 0;
				voices[i].pomakRadnogPolja_f = 0;
			}
					accFaze_uw = 0;
		 }


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

	  //EMA_LOW_Init(&low_filt, ALPHA);
  //delay_Init(&delay, 500.0f, 0.5f, 0.5f, SAMPLING_FREQ);

	#if defined  ENVELOPE
  	  	  ADSR_Init(SAMPLING_FREQ, 0.1f, 0.2f, 0.9f, 0.2f); //A, D, S, R vrijeme u s, sustain 0-1
    #endif
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	//  HAL_Delay(1000);
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
