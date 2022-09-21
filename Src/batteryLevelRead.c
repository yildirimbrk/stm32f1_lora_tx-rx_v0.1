/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name: 			 																																																											*	
*																																																																						*
*		File Name: .c																																																														*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification 																																																														*
*																																																																						*	
*		Description:																																																														*
*		Notes:																																																																	*
*																																																																						*
*	 [1] 																																																																			*
*																																																																						*
*	 [2]  																																																																		*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "batteryLevelRead.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef BatteryADCInitStruct;	
TIM_HandleTypeDef BatteryTIMInitStruct;

/* Private function prototypes -----------------------------------------------*/
//uint16_t get_BatteryADCValue(void);
/* Private function-----------------------------------------------------------*/

/**
  * @brief 	BatteryADCPortInit
  * @param 	none 
  * @retval none
	* @note 	Batarya okumasi için ADC Çevre birimi ayari saglar
	*					ADC:
	*						PA0			------> Batarya volt Okumasi
	*						PA1			------> USB volt Okumasu
  */
void BatteryADCPort_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE() ;
	__HAL_RCC_ADC1_CLK_ENABLE() ;

	GPIO_InitTypeDef Battery_GPIO_InitStruct ;
	ADC_ChannelConfTypeDef Battery_ADCChannelInitStruct ;
	
	Battery_GPIO_InitStruct.Pin    = BATTERY_PIN;
	Battery_GPIO_InitStruct.Mode   = GPIO_MODE_ANALOG ;
	Battery_GPIO_InitStruct.Pull   = GPIO_NOPULL ;
	HAL_GPIO_Init(BATTERY_PORT,&Battery_GPIO_InitStruct);
	
	BatteryADCInitStruct.Instance = ADC1 ;
	//ADC çevre birimi konfigürasyonu
	BatteryADCInitStruct.Init.ContinuousConvMode    = ENABLE ;
	BatteryADCInitStruct.Init.DataAlign						 = ADC_DATAALIGN_RIGHT ;
	BatteryADCInitStruct.Init.DiscontinuousConvMode = DISABLE ;
	BatteryADCInitStruct.Init.ExternalTrigConv      = ADC_SOFTWARE_START ;
	BatteryADCInitStruct.Init.NbrOfConversion       = 1 ;
	BatteryADCInitStruct.Init.ScanConvMode          = ENABLE ;
	HAL_ADC_Init(&BatteryADCInitStruct);
	
	//ADC_IN0
	Battery_ADCChannelInitStruct.Channel      = ADC_CHANNEL_0 ;
	Battery_ADCChannelInitStruct.Rank         = ADC_REGULAR_RANK_1;
	Battery_ADCChannelInitStruct.SamplingTime = ADC_SAMPLETIME_239CYCLES5_SMPR1ALLCHANNELS ; // 239.5 cycle
	HAL_ADC_ConfigChannel(&BatteryADCInitStruct, &Battery_ADCChannelInitStruct) ;
}

/**
  * @brief 	BatteryReadSampleTime_Init
  * @param 	none 
  * @retval none
	* @note 	Batarya okumasinin peryotlar ile yapilmasi için TIMER birimi ayari saglar
	*					Istenilen örnekleme zamani 1 sn ise; 
	*					TIMER:				 
							Ana clock frekansi = 16 MHz
							Prescaler 				 = 1000 olursa
							Prescaler cikisinda 16MHz / 1000 = 16kHz
							total counts = 500msec * f
													 = (1 sec) * 16,000
							Period			 = 16,000
  */
void BatteryReadSampleTime_Init(void)
{
	__HAL_RCC_TIM2_CLK_ENABLE();

	BatteryTIMInitStruct.Instance 						= TIM2;					
	BatteryTIMInitStruct.Init.Prescaler   		= 1000;
	BatteryTIMInitStruct.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	BatteryTIMInitStruct.Init.Period 					= 16000;
	HAL_TIM_Base_Init(&BatteryTIMInitStruct);
	
	//Enable timer-2 IRQ interrupt
	HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
	/* Enable interrupt at IRQ-Level */
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	//	HAL_TIM_Base_Start(&TIM_InitStruct);
	HAL_TIM_Base_Start_IT(&BatteryTIMInitStruct);

}

/**
  * @brief 	get_BatteryADCValue
  * @param 	none 
  * @retval 12 bitlik ADC degeri döndürür
	* @note 	Bataryanin voltaj verisini 12 bit degerinde okur ve döndürür.
  */
uint16_t get_BatteryADCValue(void)
{
	uint16_t batteryAdcValue = 0 ; 
	
	HAL_ADC_Start(&BatteryADCInitStruct) ;
	HAL_ADC_PollForConversion(&BatteryADCInitStruct, 1) ;
	batteryAdcValue = HAL_ADC_GetValue(&BatteryADCInitStruct) ;
	
	return batteryAdcValue ;
	
}

/**
  * @brief 	get_BatteryVoltValue
  * @param 	none 
  * @retval float degerinde batarya voltaj degerini döndürür
	* @note 	Bataryanin okunan 12 bitlik ADC verisini voltaj degerine dönüstürür ve döndürür.
  */
float get_BatteryVoltValue(void)
{
	uint16_t batteryAdcValue=0;
	float readPinVoltValue  =0;
	float batteryVoltValue	=0;
	
	batteryAdcValue = get_BatteryADCValue(); //	pin ADC degerini ölç
	readPinVoltValue =	(float) (MAXPINVOLT * batteryAdcValue) / MAXADCVALUE ;	// pin voltaj degerini ölç
	/*	
		Ölçülen deger pilin gerçek degeri degildir 
		pilin voltaji voltaj bölücü ile ölçülmektedir:
				Ralt=100K, Rüst=100K
				Völçülen = Vpil * Ralt/Ralt+Rüst
		pil degerine ulasmak için 2 ile çarpmak gerekmekte
	*/
	batteryVoltValue = 2 * readPinVoltValue;
	
	return batteryVoltValue;

}
