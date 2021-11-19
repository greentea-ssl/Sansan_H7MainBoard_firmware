
#ifndef _SANRAN_HPP_
#define _SANRAN_HPP_


#include "main.h"


#include "Power.hpp"
#include "RGBLED.hpp"
#include "canMotor.hpp"
#include "Buzzer.hpp"
#include "BNO055.hpp"
#include "ballSensor.hpp"
#include "dribbler.hpp"
#include "kicker.hpp"
#include "OmniWheel.hpp"
#include "controlLib.hpp"
#include "simulinkSerial.hpp"
#include "matchaSerial.hpp"


class Sanran{

public:

	Sanran();


	void setup();


	void startCycle();


	void UpdateAsync();


	void UpdateSyncHS();


	void UpdateSyncLS();


	void CAN_Rx_Callback(FDCAN_HandleTypeDef *hfdcan);


	void UART_Rx_Callback(UART_HandleTypeDef *huart);



private:


	int count;

	TIM_HandleTypeDef *htim_HS_cycle;
	TIM_HandleTypeDef *htim_LS_cycle;

	RGBLED onBrdLED;

	CanMotorIF canMotorIF;

	Buzzer buzzer;

	BNO055 bno055;

	BallSensor ballSensor;

	Dribbler dribbler;

	Kicker kicker;

	OmniWheel omni;

	MatchaSerial matcha;

	OmniWheel::Cmd_t omniCmd;

	Power power;



//	SimulinkSerial simulink;


	float deg;

	uint8_t userButton0_prev;
	uint8_t userButton1_prev;


};



#endif


