

#include "matchaSerial.hpp"


#include <stdio.h>

#include <string.h>


MatchaSerial::MatchaSerial(UART_HandleTypeDef *huart) : m_huart(huart)
{
}


void MatchaSerial::setup()
{

	m_rxBufSize = UART_BUF_SIZE;

	for(int i = 0; i < MATCHA_DATA_LENGTH; i++) m_rxBuf[i] = 0x00;

	//HAL_UART_Receive_IT(m_huart, m_rxBuf, 1);

	memset(m_rxBuf, ' ', sizeof(m_rxBuf));

	HAL_UART_Receive_DMA(huart, m_rxBuf, sizeof(m_rxBuf));

	m_rxBufSize = UART_BUF_SIZE;
	m_rxBufMask = m_rxBufSize - 1;

	m_nextWriteIndex = 0;
	m_nextReadIndex = 0;

	cmd.vel_x = 0.0;
	cmd.vel_y = 0.0;
	cmd.omega = 0.0;
	cmd.dribble = false;
	cmd.kick = false;
	cmd.chip = false;
	cmd.dribblePower = 0;
	cmd.kickPower = 0;


}


bool MatchaSerial::Update()
{

	UpdateBuffer();

	int readNum = available();
	if(readNum >= MATCHA_DATA_LENGTH)
	{
		readAllBytes();

		// Check header
		if(m_rxBytes[0] != 0xFF || m_rxBytes[1] != 0xC3) return false;

		uint8_t checkSum = 0;
		for(int i = 2; i <= 20; i++)
		{
			checkSum = checkSum ^ m_rxBytes[i];
		}
		if(m_rxBytes[21] != checkSum) return false;
		if(m_rxBytes[22] != (checkSum ^ 0xFF)) return false;


		uint32_t vel_x_int = ((uint32_t)m_rxBytes[6] << 24) | ((uint32_t)m_rxBytes[5] << 16) | ((uint32_t)m_rxBytes[4] << 8) | (uint32_t)m_rxBytes[3];
		uint32_t vel_y_int = ((uint32_t)m_rxBytes[10] << 24) | ((uint32_t)m_rxBytes[9] << 16) | ((uint32_t)m_rxBytes[8] << 8) | (uint32_t)m_rxBytes[7];
		uint32_t omega_int = ((uint32_t)m_rxBytes[14] << 24) | ((uint32_t)m_rxBytes[13] << 16) | ((uint32_t)m_rxBytes[12] << 8) | (uint32_t)m_rxBytes[11];
		uint32_t theta_fb_int = ((uint32_t)m_rxBytes[18] << 24) | ((uint32_t)m_rxBytes[17] << 16) | ((uint32_t)m_rxBytes[16] << 8) | (uint32_t)m_rxBytes[15];

		cmd.vel_x = *(float*)(&vel_x_int);
		cmd.vel_y = *(float*)(&vel_y_int);
		cmd.omega = *(float*)(&omega_int);
		cmd.theta_fb = *(float*)(&theta_fb_int);

		cmd.dribble = (m_rxBytes[19] & 0b10000000) != 0;
		cmd.kick = (m_rxBytes[19] & 0b00010000) != 0;
		cmd.chip = (m_rxBytes[19] & 0b00001000) != 0;

		cmd.dribblePower = m_rxBytes[20] >> 4;
		cmd.kickPower = m_rxBytes[20] & 0x0f;

		//printf("%6f, %6f, %6f, %6f\n", cmd.vel_x, cmd.vel_y, cmd.omega, cmd.theta_fb);

		/*
		printf("%3d > ", readNum);
		for(int i = 0; i < readNum; i++)
		{
			printf("%02x ", m_rxBytes[i]);
		}
		printf("\n");
		*/

		return true;

	}

	return false;

}


void MatchaSerial::UpdateBuffer()
{

	m_nextWriteIndex = m_rxBufSize - __HAL_DMA_GET_COUNTER(m_huart->hdmarx);

}



uint8_t MatchaSerial::readByte()
{

	uint8_t byte = m_rxBuf[m_rxBufMask & (m_nextReadIndex)];

	if((m_rxBufMask & (m_nextWriteIndex - m_nextReadIndex)) == 0) return byte;

	m_nextReadIndex++;
	if(m_nextReadIndex > m_rxBufSize - 1) m_nextReadIndex -= m_rxBufSize;

	return byte;
}



uint16_t MatchaSerial::readAllBytes()
{

	int readNum = available();
	int i;
	for(i = 0; i < readNum; i++)
	{
		m_rxBytes[i] = readByte();
	}

	return readNum;

}



uint16_t MatchaSerial::available()
{

	int16_t newByteCount = (int16_t)m_nextWriteIndex - (int16_t)m_nextReadIndex;

	if(newByteCount < 0) newByteCount += m_rxBufSize;

	return newByteCount;

}







