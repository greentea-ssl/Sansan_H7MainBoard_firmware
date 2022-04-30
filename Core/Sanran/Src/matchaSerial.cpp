

#include "matchaSerial.hpp"


#include <stdio.h>

#include <string.h>


MatchaSerial::MatchaSerial(UART_HandleTypeDef *huart) : m_huart(huart)
{

	m_timeout_enable = false;

	m_receiveState = RECEIVE_STATE_TIMEOUT;

}


bool MatchaSerial::setup(float timeout_period, float manual_timeout_period, float polling_time)
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

	normal_cmd.robot_ID = 0;
	normal_cmd.fb_x = 0.0;
	normal_cmd.fb_y = 0.0;
	normal_cmd.fb_theta = 0.0;
	normal_cmd.fb_timestamp = 0;
	normal_cmd.cmd_x = 0.0;
	normal_cmd.cmd_y = 0.0;
	normal_cmd.cmd_theta = 0.0;
	normal_cmd.cmd_vx = 0.0;
	normal_cmd.cmd_vy = 0.0;
	normal_cmd.cmd_omega = 0.0;
	normal_cmd.vel_limit = 0.0;
	normal_cmd.dribble = false;
	normal_cmd.kick = false;
	normal_cmd.chip = false;
	normal_cmd.dribblePower = 0;
	normal_cmd.kickPower = 0;
	normal_cmd.vision_error = true;


	manual_cmd.robot_ID = 0;
	manual_cmd.fb_x = 0.0;
	manual_cmd.fb_y = 0.0;
	manual_cmd.fb_theta = 0.0;
	manual_cmd.fb_timestamp = 0;
	manual_cmd.cmd_x = 0.0;
	manual_cmd.cmd_y = 0.0;
	manual_cmd.cmd_theta = 0.0;
	manual_cmd.cmd_vx = 0.0;
	manual_cmd.cmd_vy = 0.0;
	manual_cmd.cmd_omega = 0.0;
	manual_cmd.vel_limit = 0.0;
	manual_cmd.dribble = false;
	manual_cmd.kick = false;
	manual_cmd.chip = false;
	manual_cmd.dribblePower = 0;
	manual_cmd.kickPower = 0;
	manual_cmd.vision_error = true;


	m_timeout_threshold = (uint32_t)(timeout_period / polling_time);
	m_timeout_count = 0;
	m_timeout_enable = true;

	m_manual_timeout_threshold = (uint32_t)(manual_timeout_period / polling_time);
	m_manual_timeout_count = 0;

	m_receiveState = RECEIVE_STATE_NORMAL;

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


	// Update Receive State
	switch(m_receiveState)
	{
	case RECEIVE_STATE_NORMAL:
		if(m_timeout_enable)
		{
			m_timeout_count += 1;
		}
		if(m_new_data_available == true && m_prev_error_code == PARSE_ERROR_NONE \
				&& m_cmdType == CMD_TYPE_NORMAL && normal_cmd.vision_error == false)
		{
			m_timeout_count = 0;
		}

		// Change State
		if(m_timeout_count > m_timeout_threshold)
		{
			m_receiveState = RECEIVE_STATE_TIMEOUT;
		}
		if(m_new_data_available == true && m_prev_error_code == PARSE_ERROR_NONE && m_cmdType == CMD_TYPE_MANUAL)
		{
			m_receiveState = RECEIVE_STATE_MANUAL;
		}
		break;

	case RECEIVE_STATE_TIMEOUT:
		if(m_new_data_available == true && m_prev_error_code == PARSE_ERROR_NONE && normal_cmd.vision_error == false)
		{
			m_timeout_count = 0;
			m_receiveState = RECEIVE_STATE_NORMAL;
		}
		if(m_new_data_available == true && m_prev_error_code == PARSE_ERROR_NONE && m_cmdType == CMD_TYPE_MANUAL)
		{
			m_receiveState = RECEIVE_STATE_MANUAL;
		}
		break;

	case RECEIVE_STATE_MANUAL:
		m_manual_timeout_count += 1;
		if(m_new_data_available == true && m_prev_error_code == PARSE_ERROR_NONE && m_cmdType == CMD_TYPE_MANUAL)
		{
			m_manual_timeout_count = 0;
		}
		if(m_manual_timeout_count > m_manual_timeout_threshold)
		{
			m_receiveState = RECEIVE_STATE_NORMAL;
		}
		break;
	}


	return true;

}



bool MatchaSerial::parse()
{

	int idx_offset = 0;

	CommandType_TypeDef cmdType;

	// Check header
	if(m_rxBytes[idx_offset + 0] == 0xFF && m_rxBytes[idx_offset + 1] == 0xC3)
	{
		cmdType = CMD_TYPE_NORMAL;
	}
	else if(m_rxBytes[idx_offset + 0] == 0xFF && m_rxBytes[idx_offset + 1] == 0xCC)
	{
		cmdType = CMD_TYPE_MANUAL;
	}
	else
	{
		m_prev_error_code = MatchaSerial::PARSE_ERROR_HEADER;
		return false;
	}

	// Checksum
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


	if(cmdType == CMD_TYPE_NORMAL)
	{

		normal_cmd.robot_ID = m_rxBytes[idx_offset + 2] & 0x0F;

		int16_t fb_x_int      = ((int16_t)m_rxBytes[idx_offset + 4] << 8) | (int16_t)m_rxBytes[idx_offset + 3];
		int16_t fb_y_int      = ((int16_t)m_rxBytes[idx_offset + 6] << 8) | (int16_t)m_rxBytes[idx_offset + 5];
		int16_t fb_theta_int  = ((int16_t)m_rxBytes[idx_offset + 8] << 8) | (int16_t)m_rxBytes[idx_offset + 7];

		normal_cmd.fb_timestamp      = ((int16_t)m_rxBytes[idx_offset + 10] << 8) | (int16_t)m_rxBytes[idx_offset + 9];

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
			normal_cmd.vision_error = true;
		}
		else
		{
			normal_cmd.fb_x = fb_x_int * 0.001;
			normal_cmd.fb_y = fb_y_int * 0.001;
			normal_cmd.fb_theta = fb_theta_int * 2*M_PI/65536 - M_PI*0.5f;

			normal_cmd.vision_error = false;
		}

		normal_cmd.cmd_x = cmd_x_int * 0.001;
		normal_cmd.cmd_y = cmd_y_int * 0.001;
		normal_cmd.cmd_theta = cmd_theta_int * 2*M_PI/65536 - M_PI*0.5f;

		normal_cmd.cmd_vx = cmd_vx_int * 0.001;
		normal_cmd.cmd_vy = cmd_vy_int * 0.001;
		normal_cmd.cmd_omega = cmd_omega_int / 1024.0f;

		normal_cmd.vel_limit = vel_limit_int * 0.001;

		normal_cmd.dribble = (m_rxBytes[idx_offset + 25] & 0b10000000) != 0;
		normal_cmd.kick = (m_rxBytes[idx_offset + 25] & 0b00010000) != 0;
		normal_cmd.chip = (m_rxBytes[idx_offset + 25] & 0b00001000) != 0;

		normal_cmd.dribblePower = m_rxBytes[idx_offset + 26] >> 4;
		normal_cmd.kickPower = m_rxBytes[idx_offset + 26] & 0x0f;

	}
	else if(cmdType == CMD_TYPE_MANUAL)
	{

		manual_cmd.robot_ID = m_rxBytes[idx_offset + 2] & 0x0F;

		int16_t cmd_vx_int    = ((int16_t)m_rxBytes[idx_offset + 4] << 8) | (int16_t)m_rxBytes[idx_offset + 3];
		int16_t cmd_vy_int    = ((int16_t)m_rxBytes[idx_offset + 6] << 8) | (int16_t)m_rxBytes[idx_offset + 5];
		int16_t cmd_omega_int = ((int16_t)m_rxBytes[idx_offset + 8] << 8) | (int16_t)m_rxBytes[idx_offset + 7];

		manual_cmd.cmd_vx = cmd_vx_int * 0.001;
		manual_cmd.cmd_vy = cmd_vy_int * 0.001;
		manual_cmd.cmd_omega = cmd_omega_int / 1024.0f;

		manual_cmd.dribble = (m_rxBytes[idx_offset + 9] & 0b10000000) != 0;
		manual_cmd.kick = (m_rxBytes[idx_offset + 9] & 0b00010000) != 0;
		manual_cmd.chip = (m_rxBytes[idx_offset + 9] & 0b00001000) != 0;

		manual_cmd.dribblePower = m_rxBytes[idx_offset + 10] >> 4;
		manual_cmd.kickPower = m_rxBytes[idx_offset + 10] & 0x0f;

	}

	m_prev_error_code = MatchaSerial::PARSE_ERROR_NONE;

	m_cmdType = cmdType;

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
	return length;
}







