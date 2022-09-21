/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 0.1																																																														*			
*																																																																						*		
*		Module Name: 			 																																																											*	
*																																																																						*
*		File Name: main.c																																																												*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification 																																																														*
*																																																																						*	
*		Description:																																																														*
*		Notes:																																																																	*
*						02.09.2022 																																																											*
*			 Yapilacak lar:																																																												*
*		1- Acil durum butonu çalisma mantigi: NC acil durum butonu durumu 10Hz'de okunacak. buna ek olarak kesmelerle yükselen 									*
*			 kenar ve düsen kenar durumuna göre ADB statüsü degistigi bilgisi alinacak.(+)																												*
*		2- diger 4 buton basildigi sürece röleleri tetiklenecek. operatör parmagini butondan çekti mi röle geri çekilecek.(+)										*
*						04.09.2022																																																											*
*		3- batarya durumu 1sn'de bir okunacak. batarya voltaji 3.6V altina inerse batarya durumu:1'den 0'a geçecek bunun anlami 								*
*			 batarya zayif demektir. Bu durumda sinyal rölesi sürekli ötecek.																																			*
*		4- Alici vericiden 10sn'den uzun süre veri alamazsa sinyal Rölesi devreye girer. 																												*
*		5- Acil Durum rölesi ve acil stop butonu rölesi sürekli HIGH konumda olacaktir. diger röleler tetiklendiginde high durumuna geçecektir. *
*		6- Alicida sinyal rölesi, haberlesme arizasinda ve düsük batarya durumunda farkli sikliklarla tetiklenecek.															*
*		7- verici üzerinde sinyal ledi bulunacak. haberlesmeme ve düsük batarya geriliminde farkli siklikarla yanip sönecek.  									*
*		8- gyro treshold fonksiyonu disindan her saniye veri gelecek. 600 veri yani 10 dakika boyunca hareket var bilgisi gelmezse 							*
*			 sinyal rölesini farkli bir siklikla açip kapatacagiz.																																								*
*																																																																						*
*	 [1] 																																																																			*
*																																																																						*
*	 [2]  																																																																		*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private Includes -----------------------------------------------------------*/
#include "lora.h"
#include "relays_driver.h"
#include "buttonDriver.h"
#include "batteryLevelRead.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LORA_TRANSMITTER (0)
#define RECEIVER				 (0)

#define BUTTON_PORT  GPIOB 
#define BUTTON_1		 GPIO_PIN_4
#define BUTTON_2  	 GPIO_PIN_5
#define BUTTON_3		 GPIO_PIN_6
#define BUTTON_4  	 GPIO_PIN_7
#define LED_PORT     GPIOC
#define LED_PIN      GPIO_PIN_13

/* Private macro -------------------------------------------------------------*/
#define __TEST_LED_ON() HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)
#define __TEST_LED_OFF() HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)

/* Private variables ---------------------------------------------------------*/
extern SPI_HandleTypeDef LORA_SPIInitStruct;
extern TIM_HandleTypeDef BatteryTIMInitStruct;
LoRa myLoRa;
uint8_t read_data[5];
uint8_t send_data[5];
uint16_t adcValue=0;
float VoltValue=0;
float Value=0;

uint16_t batAdcValue=0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void TestLed_Init(void);


/* Private function-----------------------------------------------------------*/

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&BatteryTIMInitStruct);
	
	//voltValue = get_BatteryVoltValue();
	
}

/**
  * @brief 	main
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();
	
	/*USER CODE*/
	TestLed_Init();
	LoraPortSerial_Init();
	relays_pin_init();
	BUTTONS_Init();
	BatteryADCPort_Init();
	BatteryReadSampleTime_Init();

  myLoRa = newLoRa();
	
	myLoRa.hSPIx                 = &LORA_SPIInitStruct;
	myLoRa.CS_port               = LORA_PORT;
	myLoRa.CS_pin                = LORA_NSS_PIN;
	myLoRa.reset_port            = LORA_PORT;
	myLoRa.reset_pin             = LORA_RST_PIN;
	myLoRa.DIO0_port						 = LORA_PORT;
	myLoRa.DIO0_pin							 = LORA_DIO0_PIN;
	
	myLoRa.frequency             = 433;							  // default = 433 MHz
	myLoRa.spredingFactor        = SF_7;							// default = SF_7
	myLoRa.bandWidth			       = BW_125KHz;				  // default = BW_125KHz
	myLoRa.crcRate				       = BW_41_7KHz;						// default = CR_4_5
	myLoRa.power					       = POWER_14db;				// default = 14db
	myLoRa.overCurrentProtection = 120; 							// default = 100 mA
	myLoRa.preamble				       = 10;		  					// default = 8;
	
	
	if(LoRa_init(&myLoRa)==LORA_OK)
	{
		__TEST_LED_ON();
	}
	LoRa_startReceiving(&myLoRa);

  while (1)
  {
		batAdcValue = get_BatteryADCValue(); //	pin ADC degerini ölç
		VoltValue =	(float) (MAXPINVOLT * batAdcValue) / MAXADCVALUE ;	// pin voltaj degerini ölç
		Value = 2 * VoltValue;
	
		#if LORA_TRANSMITTER
		if(voltValue < 3.6)
			send_data[0]=0x1;
		
		else
			send_data[0]=0x0;
		
		LoRa_transmit(&myLoRa, send_data, 5, 500);
		#endif
//		// RECEIVING DATA - - - - - - - - - - - - - - - - - - - - - - - -
		#if RECEIVER
		relay1_status(read_data[0]);
		relay2_status(read_data[1]);
		relay3_status(read_data[2]);
		relay4_status(read_data[3]);
		#endif
  }
  
}





void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	#if RECEIVER
	if(GPIO_Pin == myLoRa.DIO0_pin)
	{
		LoRa_receive(&myLoRa, read_data, 5);
	}
	#endif
	
	#if LORA_TRANSMITTER
	if(GPIO_Pin == BUTTON_1)
	{
		if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_1))
			{ //check pin state 
				send_data[1]=0x1;
				
      } 
    if(!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_1))
		{
			send_data[1]=0x0;
    
		}	
	}	
	
	if(GPIO_Pin == BUTTON_2)
	{
		if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_2))
			{ //check pin state 
				send_data[2]=0x1;
      
			} 
    if(!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_2))
		{
			send_data[2]=0x0;
    
		}
	}
	
	if(GPIO_Pin == BUTTON_3)
	{
		if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_3))
			{ //check pin state 
				send_data[3]=0x1;
				
      } 
    if(!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_3))
		{
			send_data[3]=0x0;
		
		}
	}
	
	if(GPIO_Pin == BUTTON_4)
	{
		if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_4))
		{ //check pin state 
			send_data[4]=0x1;
    
		} 
    if(!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_4))
		{
			send_data[4]=0x0;
		
		}
	}
	#endif
	
}

/**
	*@brief Stm32f103 blue pill kartinin üzerindeki Led 
	*/
void TestLed_Init(void)
{
	GPIO_InitTypeDef LED_GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
  LED_GPIO_InitStruct.Pin = LED_PIN;
  LED_GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  LED_GPIO_InitStruct.Pull = GPIO_NOPULL;
  LED_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &LED_GPIO_InitStruct);
	
}


