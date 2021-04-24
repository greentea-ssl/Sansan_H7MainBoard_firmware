

#ifndef _DRIBBLER_HPP_
#define _DRIBBLER_HPP_


#include <stdint.h>
#include "stm32h7xx_hal.h"



class Dribbler{


public:

	Dribbler(TIM_HandleTypeDef *htim, uint32_t channel);

	void init();

	void write(float rate);

	void setSlow();

	void setFast();

	void setStop();



private:

	TIM_HandleTypeDef *m_htim;
	uint32_t m_channel;

	float m_outputRate;

};




#endif /* _DRIBBLER_HPP_ */
