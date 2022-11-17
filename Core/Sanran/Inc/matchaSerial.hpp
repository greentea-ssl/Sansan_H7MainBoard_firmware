

#ifndef _MATCHA_SERIAL_HPP_
#define _MATCHA_SERIAL_HPP_


// Mast be power of 2
#define UART_BUF_SIZE	(128)
#define UART_BUF_MASK	(UART_BUF_SIZE-1)


#include "stm32h7xx_hal.h"

#include "main.h"


#define MATCHA_DATA_LENGTH	(64)



class MatchaSerial
{

public:

	struct Reference_TypeDef{
		uint8_t robot_ID;
		float fb_x;
		float fb_y;
		float fb_theta;
		uint16_t fb_timestamp;
		uint16_t cmd_timestamp;
		float cmd_x;
		float cmd_y;
		float cmd_theta;
		float cmd_vx;
		float cmd_vy;
		float cmd_omega;
		float cmd_ax;
		float cmd_ay;
		float cmd_aomega;
		float vel_limit;
		float omega_limit;
		bool dribble;
		bool kick;
		bool chip;
		uint8_t dribblePower;
		uint8_t kickPower;
		bool vision_error;
	};

	Reference_TypeDef normal_cmd;
	Reference_TypeDef manual_cmd;

	typedef enum{
		PARSE_ERROR_NONE		= 0,
		PARSE_ERROR_PACKET_SIZE	= 1,
		PARSE_ERROR_HEADER		= 2,
		PARSE_ERROR_CHECK_SUM	= 3,
		PARSE_ERROR_VALUE_RANGE	= 4,
		PARSE_ERROR_NO_DATA		= 10,
	}parse_error_t;

	typedef enum{
		RECEIVE_STATE_NORMAL,
		RECEIVE_STATE_TIMEOUT,
		RECEIVE_STATE_MANUAL,
	}ReceiveState_TypeDef;

	typedef enum{
		CMD_TYPE_NORMAL,
		CMD_TYPE_MANUAL,
	}CommandType_TypeDef;


	MatchaSerial(UART_HandleTypeDef *huart, float timeout_period, float manual_timeout_period, float polling_time);

	bool setup();

	bool Update();

	bool newDataAvailable(){ return m_new_data_available; }

	bool getPrevErrorCode(){ return m_prev_error_code; }

	CommandType_TypeDef getLatestCommandType(){ return m_cmdType; }

	ReceiveState_TypeDef getReceiveState(){ return m_receiveState; }

	uint8_t m_rxBuf[UART_BUF_SIZE];
	uint8_t m_rxBytes[UART_BUF_SIZE];


private:


	uint16_t readBytes(int16_t head_index, int16_t length);

	bool parse();

	void UpdateBuffer();


	UART_HandleTypeDef *m_huart;

	// Ring buffer
	uint16_t m_rxBufSize;
	uint16_t m_rxBufMask;
	uint16_t m_nextWriteIndex;
	uint16_t m_prev_head_index;
	uint16_t m_parse_error_counter;

	parse_error_t m_prev_error_code;

	// Timeout & Receive state
	bool m_timeout_enable;
	float m_timeout_period;
	uint32_t m_timeout_count;
	uint32_t m_timeout_threshold;
	float m_manual_timeout_period;
	uint32_t m_manual_timeout_count;
	uint32_t m_manual_timeout_threshold;
	ReceiveState_TypeDef m_receiveState;
	float m_polling_time;

	bool m_new_data_available;

	// Type of latest received data
	CommandType_TypeDef m_cmdType;


};











#endif




