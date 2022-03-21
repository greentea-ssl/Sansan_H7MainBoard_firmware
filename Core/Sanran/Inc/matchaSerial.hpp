

#ifndef _MATCHA_SERIAL_HPP_
#define _MATCHA_SERIAL_HPP_


// Mast be power of 2
#define UART_BUF_SIZE	(64)
#define UART_BUF_MASK	(UART_BUF_SIZE-1)


#include "stm32h7xx_hal.h"

#include "main.h"


#define MATCHA_DATA_LENGTH	(25)



class MatchaSerial
{

public:

	struct Reference_TypeDef{
		uint8_t robot_ID;
		float fb_x;
		float fb_y;
		float fb_theta;
		float cmd_x;
		float cmd_y;
		float cmd_theta;
		float cmd_vx;
		float cmd_vy;
		float cmd_omega;
		bool dribble;
		bool kick;
		bool chip;
		uint8_t dribblePower;
		uint8_t kickPower;
		bool vision_error;
	};

	Reference_TypeDef cmd;


	typedef enum{
		PARSE_ERROR_NONE		= 0,
		PARSE_ERROR_PACKET_SIZE	= 1,
		PARSE_ERROR_HEADER		= 2,
		PARSE_ERROR_CHECK_SUM	= 3,
		PARSE_ERROR_VALUE_RANGE	= 4,
		PARSE_ERROR_NO_DATA		= 10,
	}parse_error_t;


	typedef enum{
		TIMEOUT_NONE	= 0,
		TIMEOUT_OCCURED	= 1,
	}timeout_state_t;



	MatchaSerial(UART_HandleTypeDef *huart);

	bool setup();

	bool settingTimeout(float timeout_period, float polling_time)
	{
		m_timeout_threshold = (uint32_t)(timeout_period / polling_time);

		m_timeout_count = 0;
		m_timeout_state = TIMEOUT_NONE;
		m_timeout_enable = true;
	}

	bool Update();

	bool newDataAvailable(){ return m_new_data_available; }

	bool getPrevErrorCode(){ return m_prev_error_code; }

	timeout_state_t getTimeoutState(){ return m_timeout_state; }

	uint8_t m_rxBuf[UART_BUF_SIZE];
	uint8_t m_rxBytes[UART_BUF_SIZE];


private:


	uint16_t readBytes(int16_t head_index, int16_t length);

	bool parse();

	void UpdateBuffer();


	UART_HandleTypeDef *m_huart;

	uint16_t m_rxBufSize;
	uint16_t m_rxBufMask;

	uint16_t m_nextWriteIndex;

	uint16_t m_prev_head_index;
	uint16_t m_parse_error_counter;

	parse_error_t m_prev_error_code;

	bool m_timeout_enable;
	uint32_t m_timeout_count;
	uint32_t m_timeout_threshold;
	timeout_state_t m_timeout_state;

	bool m_new_data_available;

};











#endif




