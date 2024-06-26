

#include "buzzer.hpp"

#include "math.h"

Buzzer::Buzzer(TIM_HandleTypeDef *htim, uint32_t Channel, float f_clock) : m_htim(htim), m_channel(Channel)
{

	for(int i = 0; i < 128; i++)
	{
		uint32_t scale = f_clock / 2.0 / (440.0f * pow(2.0f, (i - 69.0f) / 12.0f)) + 0.5f;
		m_prescaleTable[i] = 1;
		for(;;)
		{
			if((scale / m_prescaleTable[i]) < 65536/2)
			{
				m_periodTable[i] = scale / m_prescaleTable[i];
				break;
			}
			m_prescaleTable[i]++;
		}
		m_prescaleTable[i] -= 1;

	}

	off();
	setNoteNumber(69);

}


void Buzzer::setNoteNumber(uint8_t note)
{
	if(note > 127) return;

	__HAL_TIM_SET_COMPARE(m_htim, m_channel, m_periodTable[note]);

	m_htim->Instance->ARR = m_periodTable[note] << 1;
	m_htim->Instance->PSC = m_prescaleTable[note];

	m_htim->Instance->CNT = 0;

}


void Buzzer::on()
{
	HAL_TIM_PWM_Start(m_htim, m_channel);
}

void Buzzer::off()
{
	HAL_TIM_PWM_Stop(m_htim, m_channel);
}

void Buzzer::playTone(uint8_t note, int period_ms)
{
	on();
	setNoteNumber(note);
	HAL_Delay(period_ms);
	off();
}

void Buzzer::play(enum SoundEnum sound)
{
	switch(sound)
	{
	case SOUND_STARTUP_NORMAL:
		playTone(72+12, 100);
		playTone(74+12, 100);
		playTone(76+12, 100);
		return;

	case SOUND_STARTUP_MANUAL:
		playTone(72+12, 100);
		playTone(76+12, 100);
		playTone(79+12, 100);
		playTone(84+12, 100);
		return;

	case SOUND_STARTUP_DEBUG:
		playTone(72+12, 100);
		playTone(73+12, 100);
		playTone(74+12, 100);
		playTone(75+12, 100);
		playTone(76+12, 100);
		return;

	case SOUND_NOTIFY:
		playTone(76+12, 100);
		HAL_Delay(100);
		playTone(76+12, 100);
		HAL_Delay(100);
		return;
	}
	return;
}




