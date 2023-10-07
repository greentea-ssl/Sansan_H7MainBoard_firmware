



#include "dribbler.hpp"

#include "main.h"

#include "stdio.h"



Dribbler::Dribbler(TIM_HandleTypeDef *htim, uint32_t channel) : m_htim(htim), m_channel(channel)
{

	for(int i = 0; i < 16; i++)
	{
		// i=1: 1000us, i=15: 1100us, Linear
		m_width[i] = (i - 1) * 7.142 + 1000;
	}
	// i=0: 900us
	m_width[0] = 900;

}


void Dribbler::setup()
{

	HAL_TIM_PWM_Start(m_htim, m_channel);
	write_us(20000);
	delay_ms(1000);

	write_us(902);

/*	delay_ms(5000);

	write_us(902);

	delay_ms(5000);

	write_us(1090);
*/

//	write(0.1);
//	delay_ms(2000);
//
//	for(float rate = 0.00; rate < 0.06; rate += 0.0001)
//	{
//		delay_ms(10);
//		write(rate);
//	}
//
//	write(0.06);
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

void Dribbler::write_us(uint32_t on_time_us)
{
	const uint32_t period_us = 20000;

	m_outputRate = (float)on_time_us / period_us;

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





