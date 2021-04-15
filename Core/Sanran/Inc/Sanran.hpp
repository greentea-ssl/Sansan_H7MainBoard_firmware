
#ifndef _SANRAN_HPP_
#define _SANRAN_HPP_


#include "Power.hpp"
#include "RGBLED.hpp"



class Sanran{

public:

	Sanran();


	void UpdateAsync();


	void UpdateSyncHS();


	void UpdateSyncLS();



private:


	int count;

	Power power;

	RGBLED onBrdLED;


	float deg;



};



#endif


