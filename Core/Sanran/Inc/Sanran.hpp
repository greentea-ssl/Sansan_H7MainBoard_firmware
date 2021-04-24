
#ifndef _SANRAN_HPP_
#define _SANRAN_HPP_


#include "Power.hpp"
#include "RGBLED.hpp"
#include "canMotor.hpp"
#include "Buzzer.hpp"
#include "BNO055.hpp"
#include "dribbler.hpp"




class Sanran{

public:

	Sanran();


	void init();


	void UpdateAsync();


	void UpdateSyncHS();


	void UpdateSyncLS();


	void CAN_Rx_Callback(FDCAN_HandleTypeDef *hfdcan);



private:


	int count;

	Power power;

	RGBLED onBrdLED;

	Buzzer buzzer;

	CanMotorIF canMotorIF;

	BNO055 bno055;

	Dribbler dribbler;


	float deg;



};



#endif


