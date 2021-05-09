

#ifndef _BALL_SENSOR_HPP_
#define _BALL_SENSOR_HPP_



#include "stm32h7xx_hal.h"

#include "main.h"


class BallSensor
{
public:

	BallSensor(ADC_HandleTypeDef *hadc) : m_hadc(hadc), m_value(0.0)
{
}

	float update()
	{
		HAL_ADC_Start(m_hadc);
		HAL_ADC_PollForConversion(m_hadc, 10);
		m_value = HAL_ADC_GetValue(m_hadc) / 1024.0;
		return m_value;
	}

	float read()
	{
		return m_value;
	}

private:

	ADC_HandleTypeDef *m_hadc;

	float m_value;


};



#endif /* _BALL_SENSOR_HPP_ */
