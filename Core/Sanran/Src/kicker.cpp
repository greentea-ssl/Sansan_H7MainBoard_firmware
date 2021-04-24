


#include "kicker.hpp"



Kicker::Kicker(float cycleTime, float kickTime) : kickState(KICKSTATE_CHARGE)
{

	m_time = 0.0f;

	m_kickTime = kickTime;

	m_cycleTime = cycleTime;

}



void Kicker::kickStraight()
{

	HAL_GPIO_WritePin(KICKMODE_GPIO_Port, KICKMODE_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_SET);

	kickState = KICKSTATE_KICK;

}



void Kicker::kickChip()
{

	HAL_GPIO_WritePin(KICKMODE_GPIO_Port, KICKMODE_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_SET);

	kickState = KICKSTATE_KICK;

}


void Kicker::update()
{

	uint8_t charge_cplt = HAL_GPIO_ReadPin(DONE_GPIO_Port, DONE_Pin);

	switch(kickState)
	{
	case KICKSTATE_CHARGE:
		HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_RESET);
		if(charge_cplt == 0)
		{
			kickState = KICKSTATE_CPLT;
		}
		break;

	case KICKSTATE_CPLT:
		HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_RESET);
		break;

	case KICKSTATE_KICK:
		HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_SET);
		if(m_time >= m_kickTime)
		{
			kickState = KICKSTATE_CHARGE;
			m_time = 0.0f;
		}
		else
		{
			m_time += m_cycleTime;
		}
		break;

	default:
		break;
	}


}









