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
#include "relays_driver.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define RELAY_1 GPIO_PIN_12
#define RELAY_2 GPIO_PIN_13
#define RELAY_3 GPIO_PIN_14
#define RELAY_4 GPIO_PIN_15

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private function-----------------------------------------------------------*/


/**
  * @brief 	yok
  * @param 	yok 
  * @retval yok
	* @note 	yok
  */
void relays_pin_init(void)
{
	GPIO_InitTypeDef RELAYS_PinInit;
	
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	RELAYS_PinInit.Pin 		= RELAY_1|RELAY_2|RELAY_3|RELAY_4;
	RELAYS_PinInit.Mode 	= GPIO_MODE_OUTPUT_PP;
	RELAYS_PinInit.Speed  = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &RELAYS_PinInit);
	
}

void relay1_status(uint8_t pin_status)
{
	HAL_GPIO_WritePin(GPIOB,RELAY_1,pin_status);
	
}
void relay2_status(uint8_t pin_status)
{
	HAL_GPIO_WritePin(GPIOB,RELAY_2,pin_status);
	
}
void relay3_status(uint8_t pin_status)
{
	HAL_GPIO_WritePin(GPIOB,RELAY_3,pin_status);
	
}
void relay4_status(uint8_t pin_status)
{
	HAL_GPIO_WritePin(GPIOB,RELAY_4,pin_status);
	
}
	


