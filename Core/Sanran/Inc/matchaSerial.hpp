

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

	MatchaSerial(UART_HandleTypeDef *huart);

	bool setup();

	bool Update();

	bool getPrevErrorCode(){ return m_prev_error_code; }

	struct Reference_TypeDef{
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

};











#endif




