
#ifndef _SANRAN_HPP_
#define _SANRAN_HPP_


#include "main.h"


#include "Power.hpp"
#include "RGBLED.hpp"
#include "canMotor.hpp"
#include "Buzzer.hpp"
#include "BNO055.hpp"
#include "dribbler.hpp"
#include "kicker.hpp"
#include "OmniWheel.hpp"
#include "controlLib.hpp"
#include "simulinkSerial.hpp"


class Sanran{

public:

	Sanran();


	void startCycle();


	void UpdateAsync();


	void UpdateSyncHS();


	void UpdateSyncLS();


	void CAN_Rx_Callback(FDCAN_HandleTypeDef *hfdcan);


	void UART_Rx_Callback(UART_HandleTypeDef *huart);



private:


	int count;

	Power power;

	RGBLED onBrdLED;

	Buzzer buzzer;

	CanMotorIF canMotorIF;

	BNO055 bno055;

	Dribbler dribbler;

	Kicker kicker;

	OmniWheel omni;

	OmniWheel::Cmd_t omniCmd;

	TIM_HandleTypeDef *htim_HS_cycle;
	TIM_HandleTypeDef *htim_LS_cycle;


	SimulinkSerial simulink;


	float deg;

	uint8_t userButton0_prev;
	uint8_t userButton1_prev;


};



#endif


