

#include "simulinkSerial.hpp"


#include <stdio.h>


SimulinkSerial::SimulinkSerial(UART_HandleTypeDef *huart, uint8_t length) : m_huart(huart), m_length(length)
{

	m_dataSize = 4;

	for(int i = 0; i < m_length * m_dataSize; i++) m_rxBuf[i] = 0x00;

	for(int i = 0; i < m_length; i++) m_data[i] = 0.0f;

	HAL_UART_Receive_IT(m_huart, m_rxBuf, m_length * m_dataSize);

}


void SimulinkSerial::dataReceivedCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance != m_huart->Instance) return;

	for(int i = 0; i < m_length; i++)
	{
		m_data[i] = *(((float *)m_rxBuf) + i);
	}

	HAL_UART_Receive_IT(m_huart, m_rxBuf, m_length * m_dataSize);

}


