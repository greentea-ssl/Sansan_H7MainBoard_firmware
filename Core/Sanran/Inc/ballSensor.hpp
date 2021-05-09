

#ifndef _BALL_SENSOR_HPP_
#define _BALL_SENSOR_HPP_



#include "stm32h7xx_hal.h"

#include "main.h"


class BallSensor
{
public:

	BallSensor(ADC_HandleTypeDef *hadc) : m_hadc(hadc)
{
}

	float read()
	{
		HAL_ADC_Start(m_hadc);
		HAL_ADC_PollForConversion(m_hadc, 10);
		return HAL_ADC_GetValue(m_hadc) / 1024.0;
	}

private:

	ADC_HandleTypeDef *m_hadc;



};



#endif /* _BALL_SENSOR_HPP_ */
