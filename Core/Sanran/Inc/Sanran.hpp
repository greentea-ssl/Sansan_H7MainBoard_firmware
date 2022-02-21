
#ifndef _SANRAN_HPP_
#define _SANRAN_HPP_


#include <controlLib.hpp>
#include "main.h"


#include "power.hpp"
#include "RGBLED.hpp"
#include "canMotor.hpp"
#include "buzzer.hpp"
#include "BNO055.hpp"
#include "ballSensor.hpp"
#include "dribbler.hpp"
#include "kicker.hpp"
#include "OmniWheel.hpp"
#include "simulinkSerial.hpp"
#include "matchaSerial.hpp"
#include "debugDump.hpp"


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



	int count;


	uint32_t timeElapsed_hs_count;



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

	DebugDump<float, 40> dump;


//	SimulinkSerial simulink;


	float deg;

	uint8_t userButton0_prev;
	uint8_t userButton1_prev;


	typedef struct{
		uint32_t start_count;
		uint32_t end_count;
	}Sync_loop_timestamp_t;

	Sync_loop_timestamp_t syncHS_timestamp;
	Sync_loop_timestamp_t syncLS_timestamp;



	void dump_update();



};


#endif


