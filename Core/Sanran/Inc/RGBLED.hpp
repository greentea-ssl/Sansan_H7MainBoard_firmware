



#ifndef _RGB_LED_HPP_
#define _RGB_LED_HPP_

#include <stdint.h>
#include "stm32h7xx_hal.h"



class RGBLED{


public:

	RGBLED(TIM_HandleTypeDef *htim);

	RGBLED(TIM_HandleTypeDef *htim, uint32_t Rch, uint32_t Gch, uint32_t Bch);

	void setup();

	void setRGB(int R, int G, int B);

	void setHSV(float H, float S, float V);


private:

	TIM_HandleTypeDef *m_htim;

	uint32_t m_Rch, m_Gch, m_Bch;

	int m_R, m_G, m_B;


};



#endif /* _RGB_LED_HPP_ */



