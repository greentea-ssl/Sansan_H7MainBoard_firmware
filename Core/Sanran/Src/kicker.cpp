


#include "kicker.hpp"


extern TIM_HandleTypeDef htim4;


Kicker::Kicker(TIM_HandleTypeDef *htim, uint32_t channel)
: m_htim(htim), m_channel(channel)
{

	kickState = KICKSTATE_CHARGE;

	m_kickFlag = false;
	m_kickPower = 0;

	m_select_wait_dulation = 5;

	for(int i = 0; i < 16; i++)
	{
		m_width_straight[i] = i * 200;
		m_width_chip[i] = i * 100;
	}

}

Kicker::Kicker(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t select_wait_dulation)
: m_htim(htim), m_channel(channel)
{

	kickState = KICKSTATE_CHARGE;

	m_kickFlag = false;
	m_kickPower = 0;

	m_select_wait_dulation = select_wait_dulation;

	for(int i = 0; i < 16; i++)
	{
		m_width_straight[i] = i * 200;
		m_width_chip[i] = i * 100;
	}

}


bool Kicker::setup()
{

	HAL_TIM_PWM_Start(m_htim, m_channel);

	// Initial kick-mode : chip kick
	HAL_GPIO_WritePin(KICKMODE_GPIO_Port, KICKMODE_Pin, GPIO_PIN_SET);

	return true;
}


void Kicker::kickStraight(uint8_t power)
{
	if(power >= 16) return;

	kickStraight_width(m_width_straight[power & 0x0F]);
}



void Kicker::kickChip(uint8_t power)
{
	if(power >= 16) return;

	kickChip_width(m_width_chip[power & 0x0F]);
}



void Kicker::kickStraight_width(uint16_t width_us)
{

	if(kickState != KICKSTATE_CPLT) return;

	HAL_GPIO_WritePin(KICKMODE_GPIO_Port, KICKMODE_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_SET);

	m_kickFlag = true;
	m_kickPower = width_us;

	kickState = KICKSTATE_SELECT;
}

void Kicker::kickChip_width(uint16_t width_us)
{

	if(kickState != KICKSTATE_CPLT)
	{
		return;
	}

	HAL_GPIO_WritePin(KICKMODE_GPIO_Port, KICKMODE_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_SET);

	m_kickFlag = true;
	m_kickPower = width_us;

	kickState = KICKSTATE_SELECT;
}



void Kicker::update()
{

	uint8_t charge_cplt = HAL_GPIO_ReadPin(DONE_GPIO_Port, DONE_Pin);


	switch(kickState)
	{
	case KICKSTATE_CHARGE:
		HAL_GPIO_WritePin(KICKMODE_GPIO_Port, KICKMODE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(m_htim, m_channel, 0);
		if(charge_cplt == 0)
		{
			kickState = KICKSTATE_CPLT;
			m_kickFlag = false;
		}
		break;

	case KICKSTATE_CPLT:
		HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(m_htim, m_channel, 0);
		if(m_kickFlag)
		{
			m_select_wait_counter = 0;
			kickState = KICKSTATE_SELECT;
		}
		break;

	case KICKSTATE_SELECT:
		if(m_select_wait_counter >= m_select_wait_dulation)
		{
			m_htim->Instance->CNT = 0xfff0;
			__HAL_TIM_SET_COMPARE(m_htim, m_channel, m_kickPower);
			kickState = KICKSTATE_KICK;
		}
		m_select_wait_counter++;
		break;

	case KICKSTATE_KICK:
		HAL_GPIO_WritePin(BOOST_GPIO_Port, BOOST_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(KICK_GPIO_Port, KICK_Pin, GPIO_PIN_SET);
		if(m_htim->Instance->CNT < 0xfff0 && m_htim->Instance->CNT > m_kickPower)
		{
			kickState = KICKSTATE_CHARGE;
			__HAL_TIM_SET_COMPARE(m_htim, m_channel, 0);
		}
		break;

	default:
		__HAL_TIM_SET_COMPARE(m_htim, m_channel, 0);
		break;
	}


}









