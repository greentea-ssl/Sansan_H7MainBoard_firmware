

#include "power.hpp"


Power::Power():
	m_alivePin(ALIVE_GPIO_Port, ALIVE_Pin),
	m_enablePin(POWER_GPIO_Port, POWER_Pin),
	m_EmsPin(EMO_GPIO_Port, EMO_Pin)
{

	m_alivePin.write(0);
	m_enablePin.write(0);

}

void Power::update()
{

	m_alivePin.toggle();

}


void Power::enableSupply()
{

	m_enablePin.write(1);

}

void Power::disableSupply()
{

	m_enablePin.write(0);

}


int Power::emsPushed()
{

	return m_EmsPin.read();

}









