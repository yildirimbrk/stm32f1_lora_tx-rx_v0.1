/********************************************************************************************************************************************
*																																																																						*
*********************************************************************************************************************************************
*		System Name: 																																																														*
*																																																																						*
*		Version: 1.0																																																														*			
*																																																																						*		
*		Module Name: 			 																																																											*	
*																																																																						*
*		File Name: lora.c																																																														*
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
#include "lora.h"
/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef LORA_SPIInitStruct;

/* Private function prototypes -----------------------------------------------*/

/* Private function-----------------------------------------------------------*/

/**
  * @brief 	LoraPortSerialInit
  * @param 	none 
  * @retval none
	* @note 	Lora modülü için SPI çevre birimi ve gpio çevre birimi ayari saglar
	*					SPI1 :
	*						PA2			------> LORA_DIO0
	*						PA3			------>	LORA_RST
	*						PA4     ------> LORA_NSS
	*						PA5     ------> SPI1_LORA_SCK
	*						PA6     ------> SPI1_LORA_MISO
	*						PA7     ------> SPI1_LORA_MOSI
  */
void LoraPortSerial_Init(void)
{
	GPIO_InitTypeDef LORA_GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  
  __HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_PORT, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
	//SPI1 Pin Init
	LORA_GPIO_InitStruct.Pin   = SPI1_LORA_SCK_PIN|SPI1_LORA_MOSI_PIN;
	LORA_GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	LORA_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LORA_PORT, &LORA_GPIO_InitStruct);

	LORA_GPIO_InitStruct.Pin   = SPI1_LORA_MISO_PIN;
	LORA_GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	LORA_GPIO_InitStruct.Pull  = GPIO_NOPULL;
	HAL_GPIO_Init(LORA_PORT, &LORA_GPIO_InitStruct);
  //DIO0 Pin Init
  LORA_GPIO_InitStruct.Pin   = LORA_DIO0_PIN;
  LORA_GPIO_InitStruct.Mode  = GPIO_MODE_IT_RISING;
  LORA_GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_PORT, &LORA_GPIO_InitStruct);

  //RST, NSS Pin Init
  LORA_GPIO_InitStruct.Pin   = LORA_RST_PIN|LORA_NSS_PIN;
  LORA_GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  LORA_GPIO_InitStruct.Pull  = GPIO_NOPULL;
  LORA_GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_PORT, &LORA_GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	
	/* SPI Peripheral Init*/
	LORA_SPIInitStruct.Instance 							= SPI1;
  LORA_SPIInitStruct.Init.Mode    					= SPI_MODE_MASTER;
  LORA_SPIInitStruct.Init.Direction 				= SPI_DIRECTION_2LINES;
  LORA_SPIInitStruct.Init.DataSize  				= SPI_DATASIZE_8BIT;
  LORA_SPIInitStruct.Init.CLKPolarity 			= SPI_POLARITY_LOW;
  LORA_SPIInitStruct.Init.CLKPhase 					= SPI_PHASE_1EDGE;
  LORA_SPIInitStruct.Init.NSS 							= SPI_NSS_SOFT;
  LORA_SPIInitStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  LORA_SPIInitStruct.Init.FirstBit				  = SPI_FIRSTBIT_MSB;
  LORA_SPIInitStruct.Init.TIMode 						= SPI_TIMODE_DISABLE;
  LORA_SPIInitStruct.Init.CRCCalculation 		= SPI_CRCCALCULATION_DISABLE;
  LORA_SPIInitStruct.Init.CRCPolynomial 		= 10;
  if (HAL_SPI_Init(&LORA_SPIInitStruct) != HAL_OK)
  {
    //Error_Handler();
  }
	__HAL_SPI_ENABLE(&LORA_SPIInitStruct);
	
}
/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LORA_DIO0_PIN);
 
}

/**
  * @brief 	newLoRa
  * @param 	yok 
  * @retval  A LoRa object whith these default values:
	*										----------------------------------------
	*									  |   carrier frequency = 433 MHz        |
	*									  |    spreading factor = 7				       |
	*										|           bandwidth = 125 KHz        |
	*										| 		    coding rate = 4/5            |
	*										----------------------------------------
	* @note 	it's a constructor for LoRa structure that assign default values
	*								and pass created object (LoRa struct instanse)
  */
LoRa newLoRa()
{
	LoRa new_LoRa;

	new_LoRa.frequency             = 433 ;
	new_LoRa.spredingFactor        = SF_7 ;
	new_LoRa.bandWidth			   		 = BW_125KHz ;
	new_LoRa.crcRate               = CR_4_5 ;
	new_LoRa.power				  			 = POWER_20db ;
	new_LoRa.overCurrentProtection = 100 ;
	new_LoRa.preamble			   			 = 8 ;

	return new_LoRa;
}


/**
  * @brief 	LoRa_reset
  * @param 	LoRa* LoRa --> LoRa object handler 
  * @retval none
	* @note 	reset module
  */
void LoRa_reset(LoRa* _LoRa)
{
	HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

/**
  * @brief 	LoRa_gotoMode
  * @param 	none
  * @retval LoRa* LoRa    --> LoRa object handler
						mode	        --> select from defined modes
	* @note 	set LoRa Op mode
  */
void LoRa_gotoMode(LoRa* _LoRa, int mode)
{
	uint8_t    read;
	uint8_t    data;

	read = LoRa_read(_LoRa, RegOpMode);
	data = read;

	if(mode == SLEEP_MODE)
	{
		data = (read & 0xF8) | 0x00;
		_LoRa->current_mode = SLEEP_MODE;
	}
	else if (mode == STNBY_MODE)
	{
		data = (read & 0xF8) | 0x01;
		_LoRa->current_mode = STNBY_MODE;
	}
	else if (mode == TRANSMIT_MODE)
	{
		data = (read & 0xF8) | 0x03;
		_LoRa->current_mode = TRANSMIT_MODE;
	}
	else if (mode == RXCONTIN_MODE)
	{
		data = (read & 0xF8) | 0x05;
		_LoRa->current_mode = RXCONTIN_MODE;
	}
	else if (mode == RXSINGLE_MODE)
	{
		data = (read & 0xF8) | 0x06;
		_LoRa->current_mode = RXSINGLE_MODE;
	}

	LoRa_write(_LoRa, RegOpMode, data);
	//HAL_Delay(10);
}

/**
  * @brief 	LoRa_readReg
  * @param 	LoRa* LoRa        --> LoRa object handler
						uint8_t* address  -->	pointer to the beginning of address array
						uint16_t r_length -->	detemines number of addresse bytes that you want to send
						uint8_t* output		--> pointer to the beginning of output array
						uint16_t w_length	--> detemines number of bytes that you want to read 
  * @retval none
	* @note 	read a register(s) by an address and a length,
	*					then store value(s) at outpur array.
  */
void LoRa_readReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* output, uint16_t w_length)
{
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET) ;
	HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT) ;
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) ;
	HAL_SPI_Receive(_LoRa->hSPIx, output, w_length, RECEIVE_TIMEOUT) ;
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) ;
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET) ;

}

/**
  * @brief 	LoRa_writeReg
  * @param 	LoRa* LoRa        --> LoRa object handler
						uint8_t* address  -->	pointer to the beginning of address array
						uint16_t r_length -->	detemines number of addresse bytes that you want to send
						uint8_t* output		--> pointer to the beginning of values array
						uint16_t w_length	--> detemines number of bytes that you want to send 
  * @retval none
	* @note 	write a value(s) in a register(s) by an address
  */
void LoRa_writeReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* values, uint16_t w_length)
{
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET) ;
	HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT) ;
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) ;
	HAL_SPI_Transmit(_LoRa->hSPIx, values, w_length, TRANSMIT_TIMEOUT) ;
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) ;
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET) ;

}

/**
  * @brief 	LoRa_setFrequency
  * @param 	LoRa* LoRa        --> LoRa object handler
						int   freq        --> desired frequency in MHz unit, e.g 434 
  * @retval none
	* @note 	set carrier frequency e.g 433 MHz
  */
void LoRa_setFrequency(LoRa* _LoRa, int freq)
{
	uint8_t  data;
	uint32_t F;
	F = (freq * 524288)>>5;

	// write Msb:
	data = F >> 16;
	LoRa_write(_LoRa, RegFrMsb, data);
	HAL_Delay(5);

	// write Mid:
	data = F >> 8;
	LoRa_write(_LoRa, RegFrMid, data);
	HAL_Delay(5);

	// write Lsb:
	data = F >> 0;
	LoRa_write(_LoRa, RegFrLsb, data);
	HAL_Delay(5);
	
}

/**
  * @brief 	LoRa_setSpreadingFactor
  * @param 	LoRa* LoRa        --> LoRa object handler
						int   SP          --> desired spreading factor e.g 7
 
  * @retval none
	* @note 	set spreading factor, from 7 to 12.
  */
void LoRa_setSpreadingFactor(LoRa* _LoRa, int SF)
{
	uint8_t	data ;
	uint8_t	read ;

	if(SF>12)
		SF = 12 ;
	if(SF<7)
		SF = 7 ;

	read = LoRa_read(_LoRa, RegModemConfig2) ;
	HAL_Delay(10) ;

	data = (SF << 4) + (read & 0x0F) ;
	LoRa_write(_LoRa, RegModemConfig2, data) ;
	HAL_Delay(10) ;
}
	
/**
  * @brief 	LoRa_setPower
  * @param 	LoRa* LoRa        --> LoRa object handler
						int   power       --> desired power e.g POWER_17db 
  * @retval none
	* @note 	set power gain.
  */
void LoRa_setPower(LoRa* _LoRa, uint8_t power)
{
	LoRa_write(_LoRa, RegPaConfig, power) ;
	HAL_Delay(10) ;
}
	
/**
  * @brief 	LoRa_setOCP
  * @param 	LoRa* LoRa        --> LoRa object handler
						int   current     --> desired max currnet in mA, e.g 120
  * @retval none
	* @note 	set maximum allowed current.
  */
void LoRa_setOCP(LoRa* _LoRa, uint8_t current)
{
	uint8_t	OcpTrim = 0 ;

	if(current<45)
		current = 45 ;
	if(current>240)
		current = 240 ;

	if(current <= 120)
		OcpTrim = (current - 45)/5 ;
	else if(current <= 240)
		OcpTrim = (current + 30)/10 ;

	OcpTrim = OcpTrim + (1 << 5) ;
	LoRa_write(_LoRa, RegOcp, OcpTrim) ;
	HAL_Delay(10) ;
}

/**
  * @brief 	LoRa_setTOMsb_setCRCon
  * @param 	LoRa* LoRa        --> LoRa object handler 
  * @retval none
	* @note 	set timeout msb to 0xFF + set CRC enable.
  */
void LoRa_setTOMsb_setCRCon(LoRa* _LoRa)
{
	uint8_t read, data;

	read = LoRa_read(_LoRa, RegModemConfig2);

	data = read | 0x07;
	LoRa_write(_LoRa, RegModemConfig2, data);\
	HAL_Delay(10);
}


/**
  * @brief 	LoRa_read
  * @param 	LoRa*   LoRa        --> LoRa object handler
						uint8_t address     -->	address of the register e.g 0x1D 
  * @retval register value
	* @note 	read a register by an address
  */
uint8_t LoRa_read(LoRa* _LoRa, uint8_t address)
{
	uint8_t read_data ;
	uint8_t data_addr ;

	data_addr = address & 0x7F ;
	LoRa_readReg(_LoRa, &data_addr, 1, &read_data, 1) ;
	//HAL_Delay(5);

	return read_data;
}

/**
  * @brief 	LoRa_write
  * @param 	LoRa*   LoRa        --> LoRa object handler
						uint8_t address     -->	address of the register e.g 0x1D
						uint8_t value       --> value that you want to write 
  * @retval none
	* @note 	write a value in a register by an address
  */
void LoRa_write(LoRa* _LoRa, uint8_t address, uint8_t value)
{
	uint8_t data ;
	uint8_t addr ;

	addr = address | 0x80 ;
	data = value ;
	LoRa_writeReg(_LoRa, &addr, 1, &data, 1) ;
	//HAL_Delay(5);
}

/**
  * @brief 	LoRa_BurstWrite
  * @param 	LoRa*   LoRa        --> LoRa object handler
						uint8_t address     -->	address of the register e.g 0x1D
						uint8_t *value      --> address of values that you want to write 
  * @retval none
	* @note 	write a set of values in a register by an address respectively
  */
void LoRa_BurstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length)
{
	uint8_t addr ;
	addr = address | 0x80 ;

	//NSS = 1
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET) ;
	//say module thai I want to write in RegFiFo
	HAL_SPI_Transmit(_LoRa->hSPIx, &addr, 1, TRANSMIT_TIMEOUT) ;
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) ;
	//Write data in FiFo
	HAL_SPI_Transmit(_LoRa->hSPIx, value, length, TRANSMIT_TIMEOUT) ;
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY) ;
	//NSS = 0
	//HAL_Delay(5);
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}

/**
  * @brief 	LoRa_isvalid
  * @param 	LoRa* LoRa --> LoRa object handler 
  * @retval returns 1 if all of the values were given, otherwise returns 0
	* @note 	check the LoRa instruct values
  */
uint8_t LoRa_isvalid(LoRa* _LoRa)
{
	return 1 ;
	
}

/**
  * @brief 	LoRa_transmit
  * @param 	LoRa*    LoRa     --> LoRa object handler
			      uint8_t  data			--> A pointer to the data you wanna send
			      uint8_t	 length   --> Size of your data in Bytes
			      uint16_t timeOut	--> Timeout in milliseconds 
  * @retval 1 in case of success, 0 in case of timeout
	* @note 	Transmit data
  */
uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout)
{
	uint8_t read ;
	int mode = _LoRa->current_mode ;
	
	LoRa_gotoMode(_LoRa, STNBY_MODE) ;
	read = LoRa_read(_LoRa, RegFiFoTxBaseAddr) ;
	LoRa_write(_LoRa, RegFiFoAddPtr, read) ;
	LoRa_write(_LoRa, RegPayloadLength, length) ;
	LoRa_BurstWrite(_LoRa, RegFiFo, data, length) ;
	LoRa_gotoMode(_LoRa, TRANSMIT_MODE) ;
	
	while(1)
	{
		read = LoRa_read(_LoRa, RegIrqFlags) ;
		if((read & 0x08)!=0)
			{
				LoRa_write(_LoRa, RegIrqFlags, 0xFF) ;
				LoRa_gotoMode(_LoRa, mode) ;
				return 1 ;
			}
		else
			{
				if(--timeout==0)
				{
					LoRa_gotoMode(_LoRa, mode) ;
					return 0 ;
				}
			}
		HAL_Delay(1) ;
	}

}

/**
  * @brief 	LoRa_startReceiving
  * @param 	LoRa*    LoRa     --> LoRa object handler
  * @retval none
	* @note 	Start receiving continuously
  */
void LoRa_startReceiving(LoRa* _LoRa)
{
	LoRa_gotoMode(_LoRa, RXCONTIN_MODE) ;
}

/**
  * @brief 	LoRa_Receive
  * @param 	LoRa*    LoRa     --> LoRa object handler
						uint8_t  data			--> A pointer to the array that you want to write bytes in it
						uint8_t	 length   --> Determines how many bytes you want to read
  * @retval The number of bytes received
	* @note 	Read received data from module
  */
uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length)
{
	uint8_t read ;
	uint8_t number_of_bytes ;
	uint8_t min = 0 ;

	for(int i=0; i<length; i++)
		data[i]=0 ;

	LoRa_gotoMode(_LoRa, STNBY_MODE) ;
	read = LoRa_read(_LoRa, RegIrqFlags) ;
	if((read & 0x40) != 0)
	{
		LoRa_write(_LoRa, RegIrqFlags, 0xFF) ;
		number_of_bytes = LoRa_read(_LoRa, RegRxNbBytes) ;
		read = LoRa_read(_LoRa, RegFiFoRxCurrentAddr) ;
		LoRa_write(_LoRa, RegFiFoAddPtr, read) ;
		min = length >= number_of_bytes ? number_of_bytes : length ;
		for(int i=0; i<min; i++)
			data[i] = LoRa_read(_LoRa, RegFiFo) ;
	}
	LoRa_gotoMode(_LoRa, RXCONTIN_MODE) ;
	
  return min;

}

/**
  * @brief 	LoRa_getRSSI
  * @param 	LoRa* LoRa        --> LoRa object handler
  * @retval Returns the RSSI value of last received packet.
	* @note 	initialize and set the right setting according to LoRa sruct vars
  */
int LoRa_getRSSI(LoRa* _LoRa)
{
	uint8_t read ;
	
	read = LoRa_read(_LoRa, RegPktRssiValue) ;
	
	return -164 + read;
}

/**
  * @brief 	LoRa_init
  * @param 	LoRa* LoRa        --> LoRa object handler
  * @retval none
	* @note 	initialize and set the right setting according to LoRa sruct vars
  */
uint16_t LoRa_init(LoRa* _LoRa)
{
	uint8_t    data ;
	uint8_t    read ;

	if(LoRa_isvalid(_LoRa))
	{
		// goto sleep mode:
		LoRa_gotoMode(_LoRa, SLEEP_MODE) ;
		HAL_Delay(10) ;

		// turn on lora mode:
		read = LoRa_read(_LoRa, RegOpMode) ;
		HAL_Delay(10) ;
		data = read | 0x80 ;
		LoRa_write(_LoRa, RegOpMode, data) ;
		HAL_Delay(100) ;

		// set frequency:
		LoRa_setFrequency(_LoRa, _LoRa->frequency) ;

		// set output power gain:
		LoRa_setPower(_LoRa, _LoRa->power) ;

		// set over current protection:
		LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection) ;

		// set LNA gain:
		LoRa_write(_LoRa, RegLna, 0x23) ;

		// set spreading factor, CRC on, and Timeout Msb:
		LoRa_setTOMsb_setCRCon(_LoRa) ;
		LoRa_setSpreadingFactor(_LoRa, _LoRa->spredingFactor) ;

		// set Timeout Lsb:
		LoRa_write(_LoRa, RegSymbTimeoutL, 0xFF) ;

		// set bandwidth, coding rate and expilicit mode:
		// 8 bit RegModemConfig --> | X | X | X | X | X | X | X | X |
		//       bits represent --> |   bandwidth   |     CR    |I/E|
		data = 0 ;
		data = (_LoRa->bandWidth << 4) + (_LoRa->crcRate << 1) ;
		LoRa_write(_LoRa, RegModemConfig1, data) ;

	// set preamble:
		LoRa_write(_LoRa, RegPreambleMsb, _LoRa->preamble >> 8) ;
		LoRa_write(_LoRa, RegPreambleLsb, _LoRa->preamble >> 0) ;

	// DIO mapping:   --> DIO: RxDone
		read = LoRa_read(_LoRa, RegDioMapping1) ;
		data = read | 0x3F ;
		LoRa_write(_LoRa, RegDioMapping1, data) ;

	// goto standby mode:
		LoRa_gotoMode(_LoRa, STNBY_MODE) ;
		_LoRa->current_mode = STNBY_MODE ;
		HAL_Delay(10) ;

		read = LoRa_read(_LoRa, RegVersion) ;
		if(read == 0x12)
			return LORA_OK ;
		else
			return LORA_NOT_FOUND ;
	}
	
	else 
		return LORA_UNAVAILABLE ;
	
}
