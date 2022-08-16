

#ifndef _DEBUG_DUMP_HPP_
#define _DEBUG_DUMP_HPP_


#include "stm32h7xx_hal.h"

#include <string.h>


template <typename bufType, uint32_t bufLength>
class DebugDump
{

public:

	DebugDump(SPI_HandleTypeDef *hspi)
	{

		m_hspi = hspi;

		memset(dataBuf, 0x00, sizeof(dataBuf));

	}

	void setValue(uint32_t index, bufType value)
	{
		if(index >= bufLength) return;

		dataBuf[index + 2] = value;

	}

	void send()
	{

		memcpy(sendBuf, dataBuf, sizeof(sendBuf));

		uint8_t *p = (uint8_t*)sendBuf;

		p[sizeof(bufType) * 2 - 2] = 0xFF;
		p[sizeof(bufType) * 2 - 1] = 0xAA;

		uint8_t* pSend = &(p[sizeof(bufType) * 2 - 2]);

		HAL_SPI_Transmit_IT(m_hspi, pSend, sizeof(bufType) * bufLength + 2);

	}


private:


	bufType dataBuf[bufLength + 2];
	bufType sendBuf[bufLength + 2];

	SPI_HandleTypeDef *m_hspi;

};






#endif /* _DEBUG_DUMP_HPP_ */
