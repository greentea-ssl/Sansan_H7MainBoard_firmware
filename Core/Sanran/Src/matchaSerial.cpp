

#include "matchaSerial.hpp"


#include <stdio.h>

#include <string.h>


MatchaSerial::MatchaSerial(UART_HandleTypeDef *huart) : m_huart(huart)
{
}


bool MatchaSerial::setup()
{

	HAL_StatusTypeDef status;

	m_rxBufSize = UART_BUF_SIZE;

	for(int i = 0; i < MATCHA_DATA_LENGTH; i++) m_rxBuf[i] = 0x00;

	//HAL_UART_Receive_IT(m_huart, m_rxBuf, 1);

	memset(m_rxBuf, ' ', sizeof(m_rxBuf));

	status = HAL_UART_Receive_DMA(m_huart, m_rxBuf, sizeof(m_rxBuf));

	m_rxBufSize = UART_BUF_SIZE;
	m_rxBufMask = m_rxBufSize - 1;

	m_nextWriteIndex = 0;

	m_prev_head_index = 0;
	m_parse_error_counter = 0;

	cmd.vel_x = 0.0;
	cmd.vel_y = 0.0;
	cmd.omega = 0.0;
	cmd.dribble = false;
	cmd.kick = false;
	cmd.chip = false;
	cmd.dribblePower = 0;
	cmd.kickPower = 0;

	return (status == HAL_OK);
}


bool MatchaSerial::Update()
{

	UpdateBuffer();

	//int readNum = available();

	int16_t head_index = 0;

	int16_t updated_size = ((m_nextWriteIndex - MATCHA_DATA_LENGTH) - m_prev_head_index) & m_rxBufMask;

	if(updated_size >= MATCHA_DATA_LENGTH)
	{
		head_index = ((updated_size / MATCHA_DATA_LENGTH) * MATCHA_DATA_LENGTH + m_prev_head_index) & m_rxBufMask;
	}
	else
	{
		head_index = m_prev_head_index;
	}

	readBytes(head_index, MATCHA_DATA_LENGTH);
	m_prev_head_index = head_index;


	if(parse())
	{
		m_parse_error_counter = 0;
	}
	else
	{
		// Parse Failed
		m_parse_error_counter += 1;
		if(m_parse_error_counter >= 3)
		{
			m_prev_head_index = (m_nextWriteIndex - MATCHA_DATA_LENGTH) & m_rxBufMask;
		}
	}

	return true;

}



bool MatchaSerial::parse()
{

	int idx_offset = 0;

	// Check header
	if(m_rxBytes[idx_offset + 0] != 0xFF || m_rxBytes[idx_offset + 1] != 0xC3)
	{
		m_prev_error_code = MatchaSerial::PARSE_ERROR_HEADER;
		return false;
	}

	uint8_t checkSum = 0;
	for(int i = 2; i <= 20; i++)
	{
		checkSum = checkSum ^ m_rxBytes[idx_offset + i];
	}
	if(m_rxBytes[idx_offset + 21] != checkSum)
	{
		m_prev_error_code = MatchaSerial::PARSE_ERROR_CHECK_SUM;
		return false;
	}
	if(m_rxBytes[idx_offset + 22] != (checkSum ^ 0xFF))
	{
		m_prev_error_code = PARSE_ERROR_CHECK_SUM;
		return false;
	}

#if 1

	uint32_t vel_x_int = ((uint32_t)m_rxBytes[idx_offset + 6] << 24) | ((uint32_t)m_rxBytes[idx_offset + 5] << 16) | ((uint32_t)m_rxBytes[idx_offset + 4] << 8) | (uint32_t)m_rxBytes[idx_offset + 3];
	uint32_t vel_y_int = ((uint32_t)m_rxBytes[idx_offset + 10] << 24) | ((uint32_t)m_rxBytes[idx_offset + 9] << 16) | ((uint32_t)m_rxBytes[idx_offset + 8] << 8) | (uint32_t)m_rxBytes[idx_offset + 7];
	uint32_t omega_int = ((uint32_t)m_rxBytes[idx_offset + 14] << 24) | ((uint32_t)m_rxBytes[idx_offset + 13] << 16) | ((uint32_t)m_rxBytes[idx_offset + 12] << 8) | (uint32_t)m_rxBytes[idx_offset + 11];
	uint32_t theta_fb_int = ((uint32_t)m_rxBytes[idx_offset + 18] << 24) | ((uint32_t)m_rxBytes[idx_offset + 17] << 16) | ((uint32_t)m_rxBytes[idx_offset + 16] << 8) | (uint32_t)m_rxBytes[idx_offset + 15];

	cmd.vel_x = *(float*)(&vel_x_int);
	cmd.vel_y = *(float*)(&vel_y_int);
	cmd.omega = *(float*)(&omega_int);
	cmd.theta_fb = *(float*)(&theta_fb_int);

	cmd.dribble = (m_rxBytes[idx_offset + 19] & 0b10000000) != 0;
	cmd.kick = (m_rxBytes[idx_offset + 19] & 0b00010000) != 0;
	cmd.chip = (m_rxBytes[idx_offset + 19] & 0b00001000) != 0;

	cmd.dribblePower = m_rxBytes[idx_offset + 20] >> 4;
	cmd.kickPower = m_rxBytes[idx_offset + 20] & 0x0f;

#endif

	m_prev_error_code = MatchaSerial::PARSE_ERROR_NONE;

	return true;

}



void MatchaSerial::UpdateBuffer()
{

	m_nextWriteIndex = m_rxBufSize - __HAL_DMA_GET_COUNTER(m_huart->hdmarx);

}


uint16_t MatchaSerial::readBytes(int16_t head_index, int16_t length)
{
	for(int i = 0; i < length; i++)
	{
		m_rxBytes[i] = m_rxBuf[m_rxBufMask & (i + head_index)];
	}
}







