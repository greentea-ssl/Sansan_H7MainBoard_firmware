

#include "matchaSerial.hpp"


#include <stdio.h>

#include <string.h>


MatchaSerial::MatchaSerial(UART_HandleTypeDef *huart) : m_huart(huart)
{

	m_timeout_enable = false;

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

	cmd.robot_ID = 0;

	cmd.fb_x = 0.0;
	cmd.fb_y = 0.0;
	cmd.fb_theta = 0.0;
	cmd.fb_timestamp = 0;
	cmd.cmd_x = 0.0;
	cmd.cmd_y = 0.0;
	cmd.cmd_theta = 0.0;
	cmd.cmd_vx = 0.0;
	cmd.cmd_vy = 0.0;
	cmd.cmd_omega = 0.0;
	cmd.vel_limit = 0.0;

	cmd.dribble = false;
	cmd.kick = false;
	cmd.chip = false;
	cmd.dribblePower = 0;
	cmd.kickPower = 0;

	cmd.vision_error = true;

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
		m_new_data_available = true;
	}
	else
	{
		head_index = m_prev_head_index;
		m_new_data_available = false;
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


	// update timeout
	if(m_timeout_enable && m_timeout_state == TIMEOUT_NONE)
	{
		m_timeout_count += 1;
		if(m_timeout_count > m_timeout_threshold)
		{
			m_timeout_state = TIMEOUT_OCCURED;
		}
	}
	if(m_new_data_available == true && m_prev_error_code == PARSE_ERROR_NONE && cmd.vision_error == false)
	{
		m_timeout_count = 0;
		m_timeout_state = TIMEOUT_NONE;
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
	for(int i = 2; i <= 26; i++)
	{
		checkSum = checkSum ^ m_rxBytes[idx_offset + i];
	}
	if(m_rxBytes[idx_offset + 27] != checkSum)
	{
		m_prev_error_code = MatchaSerial::PARSE_ERROR_CHECK_SUM;
		return false;
	}
	if(m_rxBytes[idx_offset + 28] != (checkSum ^ 0xFF))
	{
		m_prev_error_code = PARSE_ERROR_CHECK_SUM;
		return false;
	}

#if 1


	cmd.robot_ID = m_rxBytes[idx_offset + 2] & 0x0F;

	int16_t fb_x_int      = ((int16_t)m_rxBytes[idx_offset + 4] << 8) | (int16_t)m_rxBytes[idx_offset + 3];
	int16_t fb_y_int      = ((int16_t)m_rxBytes[idx_offset + 6] << 8) | (int16_t)m_rxBytes[idx_offset + 5];
	int16_t fb_theta_int  = ((int16_t)m_rxBytes[idx_offset + 8] << 8) | (int16_t)m_rxBytes[idx_offset + 7];

	cmd.fb_timestamp      = ((int16_t)m_rxBytes[idx_offset + 10] << 8) | (int16_t)m_rxBytes[idx_offset + 9];

	int16_t cmd_x_int     = ((int16_t)m_rxBytes[idx_offset + 12] << 8) | (int16_t)m_rxBytes[idx_offset + 11];
	int16_t cmd_y_int     = ((int16_t)m_rxBytes[idx_offset + 14] << 8) | (int16_t)m_rxBytes[idx_offset + 13];
	int16_t cmd_theta_int = ((int16_t)m_rxBytes[idx_offset + 16] << 8) | (int16_t)m_rxBytes[idx_offset + 15];

	int16_t cmd_vx_int    = ((int16_t)m_rxBytes[idx_offset + 18] << 8) | (int16_t)m_rxBytes[idx_offset + 17];
	int16_t cmd_vy_int    = ((int16_t)m_rxBytes[idx_offset + 20] << 8) | (int16_t)m_rxBytes[idx_offset + 19];
	int16_t cmd_omega_int = ((int16_t)m_rxBytes[idx_offset + 22] << 8) | (int16_t)m_rxBytes[idx_offset + 21];

	int16_t vel_limit_int = ((int16_t)m_rxBytes[idx_offset + 24] << 8) | (int16_t)m_rxBytes[idx_offset + 23];

	// Detect vision error
	if(fb_x_int == 0x7FFF || fb_y_int == 0x7FFF || fb_theta_int == 0x7FFF)
	{
		cmd.vision_error = true;
	}
	else
	{
		cmd.fb_x = fb_x_int * 0.001;
		cmd.fb_y = fb_y_int * 0.001;
		cmd.fb_theta = fb_theta_int * 2*M_PI/65536 - M_PI*0.5f;

		cmd.vision_error = false;
	}

	cmd.cmd_x = cmd_x_int * 0.001;
	cmd.cmd_y = cmd_y_int * 0.001;
	cmd.cmd_theta = cmd_theta_int * 2*M_PI/65536 - M_PI*0.5f;

	cmd.cmd_vx = cmd_vx_int * 0.001;
	cmd.cmd_vy = cmd_vy_int * 0.001;
	cmd.cmd_omega = cmd_omega_int / 1024.0f;

	cmd.vel_limit = vel_limit_int * 0.001;

	cmd.dribble = (m_rxBytes[idx_offset + 25] & 0b10000000) != 0;
	cmd.kick = (m_rxBytes[idx_offset + 25] & 0b00010000) != 0;
	cmd.chip = (m_rxBytes[idx_offset + 25] & 0b00001000) != 0;

	cmd.dribblePower = m_rxBytes[idx_offset + 26] >> 4;
	cmd.kickPower = m_rxBytes[idx_offset + 26] & 0x0f;

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







