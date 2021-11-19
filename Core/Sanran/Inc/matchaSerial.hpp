

#ifndef _MATCHA_SERIAL_HPP_
#define _MATCHA_SERIAL_HPP_


#define UART_BUF_SIZE	(32)
#define UART_BUF_MASK	(UART_BUF_SIZE-1)


#include "stm32h7xx_hal.h"

#include "main.h"


#define MATCHA_DATA_LENGTH	(23)





class MatchaSerial
{

public:

	MatchaSerial(UART_HandleTypeDef *huart);

	bool setup();

	bool Update();

	struct Reference_TypeDef{
		float vel_x;
		float vel_y;
		float omega;
		float theta_fb;
		bool dribble;
		bool kick;
		bool chip;
		uint8_t dribblePower;
		uint8_t kickPower;
	};

	Reference_TypeDef cmd;


private:

	uint8_t readByte();

	uint16_t readAllBytes();

	uint16_t available();

	void UpdateBuffer();

	UART_HandleTypeDef *m_huart;

	uint16_t m_rxBufSize;
	uint16_t m_rxBufMask;

	uint16_t m_nextWriteIndex;
	uint16_t m_nextReadIndex;

	uint8_t m_rxBuf[UART_BUF_SIZE];

	uint8_t m_rxBytes[UART_BUF_SIZE];

	uint16_t m_readByteCount;


};











#endif




