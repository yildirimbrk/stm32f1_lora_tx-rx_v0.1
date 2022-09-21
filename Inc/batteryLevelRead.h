/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name:																																																														*	
*																																																																						*
*		File Name: 																																																															*
*																																																																						*
*		Lead Author: Burak YILDIRIM.																																																						*
*																																																																						*
*		Modification Dates: 																																																										*
*																																																																						*	
*		Description:																																																														*
*																																																																						*
																																																																						*	
*																																																																						*
* 																																																																					*
*		Notes:																																																																	*
*																																																																						*
*	 [1]																																																																			*
*	 																																																																					*
*	 [2]																																																										    							*
*********************************************************************************************************************************************
*																																																																						*	
********************************************************************************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __batteryRead
#define __batteryRead

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define BATTERY_PORT 						GPIOA
#define BATTERY_PIN 						GPIO_PIN_0
#define USB_PIN									GPIO_PIN_1
#define MAXADCVALUE 						4096
#define MAXPINVOLT 			(float) 3.3
#define FULLUSBVOLT 		(float)	5.0
#define FULLBATTERYVOLT (float) 4.2
	
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void BatteryADCPort_Init(void);
void BatteryReadSampleTime_Init(void);
uint16_t get_BatteryADCValue(void);
float get_BatteryVoltValue(void);



#endif
