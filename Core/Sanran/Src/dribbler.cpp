



#include "dribbler.hpp"

#include "main.h"

#include "stdio.h"



Dribbler::Dribbler(TIM_HandleTypeDef *htim, uint32_t channel) : m_htim(htim), m_channel(channel)
{

}


void Dribbler::setup()
{

	HAL_TIM_PWM_Start(m_htim, m_channel);

	write(0.1);
	delay_ms(2000);

	for(float rate = 0.09; rate < 0.11; rate += 0.001)
	{
		delay_ms(10);
		write(rate);
	}

	write(0.1318);
/*
	for(float rate = 0.09; rate < 0.1323; rate += 0.0001)
	{
		delay_ms(5);
		write(rate);
	}
*/


}


void Dribbler::write(float rate)
{

	m_outputRate = rate;

	__HAL_TIM_SET_COMPARE(m_htim, m_channel, m_outputRate * m_htim->Init.Period);

}



void Dribbler::setSlow()
{
	write(0.1318);
}

void Dribbler::setFast()
{
	write(0.135);
}

void Dribbler::setStop()
{
	write(0.1);
}





