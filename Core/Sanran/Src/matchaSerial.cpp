

#include "matchaSerial.hpp"


#include <stdio.h>


MatchaSerial::MatchaSerial(UART_HandleTypeDef *huart) : m_huart(huart)
{

	for(int i = 0; i < MATCHA_DATA_LENGTH; i++) m_rxBuf[i] = 0x00;

	HAL_UART_Receive_IT(m_huart, m_rxBuf, 1);

}


void MatchaSerial::dataReceivedCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance != m_huart->Instance) return;


	HAL_UART_Receive_IT(m_huart, m_rxBuf, 1);

}


