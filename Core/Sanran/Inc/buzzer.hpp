

#ifndef _BUZZER_HPP_
#define _BUZZER_HPP_


#include "stm32h7xx_hal.h"


class Buzzer
{

public:

	Buzzer(TIM_HandleTypeDef *htim, uint32_t Channel, float f_clock);

	void setNoteNumber(uint8_t note);

	void on();

	void off();


	void sound_startup();


private:

	TIM_HandleTypeDef *m_htim;

	uint32_t m_channel;

	float m_f_clock;

	uint8_t m_note;

	uint16_t m_prescaleTable[128];
	uint16_t m_periodTable[128];

};





#endif /* _BUZZER_HPP_ */
