



#ifndef _KICKER_HPP_
#define _KICKER_HPP_

#include "stm32h7xx_hal.h"

#include "main.h"



class Kicker
{

public:

	Kicker(TIM_HandleTypeDef *htim, uint32_t channel);

	bool setup();

	void kickStraight(uint8_t power);

	void kickChip(uint8_t power);

	void kickStraight_width(uint16_t width_us);

	void kickChip_width(uint16_t width_us);

	void update();

	bool chargeCompleted(){ return (kickState == KICKSTATE_CPLT); }


	enum KickState_e{
			KICKSTATE_CHARGE,
			KICKSTATE_CPLT,
			KICKSTATE_KICK,
	};

	enum KickState_e kickState;

private:


	uint32_t kickPower;

	TIM_HandleTypeDef *m_htim;
	uint32_t m_channel;

	uint32_t m_width_chip[16];
	uint32_t m_width_straight[16];


};








#endif /* _KICKER_HPP_ */




