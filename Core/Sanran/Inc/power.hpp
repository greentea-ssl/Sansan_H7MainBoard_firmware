


#ifndef _POWER_HPP_
#define _POWER_HPP_



 /* Use pin labels */
#include "main.h"


#include "DigitalIO.hpp"



class Power{

public:

	Power();


	void update();


	void enableSupply();

	void disableSupply();


	int emsPushed();


private:

	DigitalOut m_alivePin;
	DigitalOut m_enablePin;
	DigitalIn  m_EmsPin;


};




#endif



