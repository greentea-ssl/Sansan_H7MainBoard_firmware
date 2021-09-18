

#ifndef _MATCHA_SERIAL_HPP_
#define _MATCHA_SERIAL_HPP_


#include "stm32h7xx_hal.h"

#include "main.h"


#define MATCHA_DATA_LENGTH	(23)


class MatchaSerial
{

public:

	MatchaSerial(UART_HandleTypeDef *huart);

	void dataReceivedCallback(UART_HandleTypeDef *huart);

	uint8_t m_rxBuf[MATCHA_DATA_LENGTH];

private:

	UART_HandleTypeDef *m_huart;


};











#endif




