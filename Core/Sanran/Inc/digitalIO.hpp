



#ifndef _DIGITAL_IO_HPP_
#define _DIGITAL_IO_HPP_

#include <stdint.h>
#include "stm32h7xx_hal.h"



class DigitalIn{

public:

	DigitalIn(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


	int read(void);


private:

	GPIO_TypeDef* m_GPIOx;
	uint16_t m_GPIO_Pin;


};


class DigitalOut{

public:

	DigitalOut(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);


	void write(int value);


	void toggle(void);


	int read(void);



private:

	GPIO_TypeDef* m_GPIOx;
	uint16_t m_GPIO_Pin;


};



#endif /* _DIGITAL_IO_HPP_ */



