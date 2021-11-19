


#include "RGBLED.hpp"


#include "math.h"



RGBLED::RGBLED(TIM_HandleTypeDef *htim)
: m_htim(htim)
{

	m_Rch = TIM_CHANNEL_1;
	m_Gch = TIM_CHANNEL_2;
	m_Bch = TIM_CHANNEL_3;

}


RGBLED::RGBLED(TIM_HandleTypeDef *htim, uint32_t Rch, uint32_t Gch, uint32_t Bch)
: m_htim(htim), m_Rch(Rch), m_Gch(Gch), m_Bch(Bch)
{
}


/**
 * @fn void setup()
 * @brief Peripheral setting
 *
 */
bool RGBLED::setup()
{

	uint8_t status = 0;

	status |= HAL_TIM_PWM_Start_IT(m_htim, m_Rch);
	status |= HAL_TIM_PWM_Start_IT(m_htim, m_Gch);
	status |= HAL_TIM_PWM_Start_IT(m_htim, m_Bch);

	setRGB(0, 0, 0);

	return status == 0;
}


/**
 * @fn void setRGB(int, int, int)
 * @brief
 *
 * @param R
 * @param G
 * @param B
 */
void RGBLED::setRGB(int R, int G, int B)
{

	__HAL_TIM_SET_COMPARE(m_htim, m_Rch, m_R = R);
	__HAL_TIM_SET_COMPARE(m_htim, m_Gch, m_G = G);
	__HAL_TIM_SET_COMPARE(m_htim, m_Bch, m_B = B);

}



/**
 * @fn void setHSV(float, float, float)
 * @brief Wikipediaからコピペ
 *
 * @param H: 0.0 ~ 1.0
 * @param S: 0.0 ~ 1.0
 * @param V: 0.0 ~ 1.0
 */
void RGBLED::setHSV(float H, float S, float V)
{

	float r = V;
	float g = V;
	float b = V;
	if (S > 0.0f) {
	    H *= 6.0f;
	    int i = (int) H;
	    float f = H - (float) i;
	    switch (i) {
	        default:
	        case 0:
	            g *= 1 - S * (1 - f);
	            b *= 1 - S;
	            break;
	        case 1:
	            r *= 1 - S * f;
	            b *= 1 - S;
	            break;
	        case 2:
	            r *= 1 - S;
	            b *= 1 - S * (1 - f);
	            break;
	        case 3:
	            r *= 1 - S;
	            g *= 1 - S * f;
	            break;
	        case 4:
	            r *= 1 - S * (1 - f);
	            g *= 1 - S;
	            break;
	        case 5:
	            g *= 1 - S;
	            b *= 1 - S * f;
	            break;
	    }
	}

	setRGB(r*255, g*255, b*255);

}



