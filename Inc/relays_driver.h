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
#ifndef __relays_driver
#define __relays_driver

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void relays_pin_init(void);
void relay1_status(uint8_t pin_status);
void relay2_status(uint8_t pin_status);
void relay3_status(uint8_t pin_status);
void relay4_status(uint8_t pin_status);
/* Private defines -----------------------------------------------------------*/


#endif
