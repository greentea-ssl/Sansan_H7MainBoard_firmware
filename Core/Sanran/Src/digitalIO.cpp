


#include "digitalIO.hpp"



DigitalIn::DigitalIn(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin): m_GPIOx(GPIOx), m_GPIO_Pin(GPIO_Pin)
{
}


int DigitalIn::read(void)
{
	return HAL_GPIO_ReadPin(m_GPIOx, m_GPIO_Pin);
}


DigitalOut::DigitalOut(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin): m_GPIOx(GPIOx), m_GPIO_Pin(GPIO_Pin)
{
}


void DigitalOut::write(int value)
{
	HAL_GPIO_WritePin(m_GPIOx, m_GPIO_Pin, (GPIO_PinState)value);
}


int DigitalOut::read(void)
{
	return HAL_GPIO_ReadPin(m_GPIOx, m_GPIO_Pin);
}


void DigitalOut::toggle(void)
{
	HAL_GPIO_TogglePin(m_GPIOx, m_GPIO_Pin);
}




