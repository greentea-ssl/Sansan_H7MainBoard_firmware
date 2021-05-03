

#ifndef _SIMULINK_SERIAL_HPP_
#define _SIMULINK_SERIAL_HPP_


#include "stm32h7xx_hal.h"

#include "main.h"



class SimulinkSerial
{

public:

	SimulinkSerial(UART_HandleTypeDef *huart, uint8_t length);

	void dataReceivedCallback(UART_HandleTypeDef *huart);

	uint8_t m_rxBuf[16];

	float m_data[4];

private:

	UART_HandleTypeDef *m_huart;

	uint8_t m_length;

	uint8_t m_dataSize;


};











#endif




