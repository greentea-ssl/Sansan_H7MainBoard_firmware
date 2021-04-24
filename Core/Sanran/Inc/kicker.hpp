



#ifndef _KICKER_HPP_
#define _KICKER_HPP_

#include "stm32h7xx_hal.h"

#include "main.h"



class Kicker
{

public:

	Kicker(float cycleTime, float kickTime);


	void kickStraight();

	void kickChip();

	void update();



	bool chargeCompleted(){ return (kickState == KICKSTATE_CPLT); }



private:

	enum KickState_e{
			KICKSTATE_CHARGE,
			KICKSTATE_CPLT,
			KICKSTATE_KICK,
	};

	enum KickState_e kickState;


	float m_time;

	float m_kickTime;

	float m_cycleTime;




};








#endif /* _KICKER_HPP_ */




