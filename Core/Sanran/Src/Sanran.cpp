


#include "Sanran.hpp"



#include <stdio.h>
#include "main.h"


#include "stm32h753xx.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern FDCAN_HandleTypeDef hfdcan2;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_uart5_rx;


/* Debug variables */

volatile float omega_w = 0.0;
volatile float omega_w_ref = 0.0;
volatile float Iq_ref = 0.0;
volatile float lpf_test = 0.0f;

volatile float odo_x = 0.0f;
volatile float odo_y = 0.0f;
volatile float odo_theta = 0.0f;

volatile float wheel_theta[4] = {0};


/**
 * @fn  Sanran()
 * @brief Constructor of Sanran class
 *
 */
Sanran::Sanran()
	: htim_HS_cycle(&htim12),
	  htim_LS_cycle(&htim13),
	  onBrdLED(&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3),
	  canMotorIF(&hfdcan2),
	  buzzer(&htim2, TIM_CHANNEL_1, 240E+6),
	  bno055(&hi2c2),
	  ballSensor(&hadc1),
	  dribbler(&htim1, TIM_CHANNEL_1),
	  kicker(&htim4, TIM_CHANNEL_1),
	  omni(OmniWheel::TYPE_WORLD_POSITION, &canMotorIF),
	  matcha(&huart5),
	  dump(&huart1)
{

	// Operation mode is normal mode
	opeMode = OPE_MODE_NORMAL;

}



void Sanran::setup()
{

	bool boolStatus;

	printf("Hello.\n\n");

	buzzer.sound_startup();

	delay_ms(1000);

	printf("********** Initialize ********************\n\n");


	/***** Setting On board LED *****/
	printf("\n Setting On board LED ...  \n");
	boolStatus = onBrdLED.setup();
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}

	/***** Setting CAN motor IF *****/
	printf("\n Setting CAN motor IF ...  \n");
	boolStatus = canMotorIF.setup();
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}

	/***** Setting BNO055 *****/
	printf("\n Setting BNO055 ...  \n");
	boolStatus = bno055.setup();
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}

	/***** Setting Omni Wheel *****/
	printf("\n Setting Omni Wheel ... \n");
	omni.setup();
	printf("\t\t\t\t[Completed]\n");

	/***** Setting Kicker *****/
	printf("\n Setting Kicker ...  \n");
	boolStatus = kicker.setup();
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}

	/***** Setting Matcha Serial *****/
	printf("\n Setting Matcha Serial ... \n");
	boolStatus = matcha.setup(0.1, 5.0, 0.01);
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}

	timeElapsed_hs_count = 0;

	onBrdLED.setRGB(0, 0, 0);


	deg = 0.0;

	userButton0_prev = 1;
	userButton1_prev = 1;

	power.enableSupply();


	dribbler.setup();
	//dribbler.setStop();
	//dribbler.setFast();
	//dribbler.setSlow();


	// wait for BLDC sensor calibration
	printf("\n Waiting for MD calibration ... \n");
	for(int i = 0; i < 8; i++)
	{
		delay_ms(500);
		buzzer.sound_singleTone(72+12, 100);
	}


	printf("\n********** Start ********************\n");

}


void Sanran::startCycle()
{

	HAL_TIM_Base_Start_IT(htim_HS_cycle);
	HAL_TIM_Base_Start_IT(htim_LS_cycle);

}



/**
 * @fn void UpdateAsync()
 * @brief　非同期ループ(タイミングクリティカルな制御は禁止)
 *
 */
void Sanran::UpdateAsync()
{
}

/**
 * @fn void UpdateSyncHS()
 * @brief 制御用高速ループ
 *
 */
void Sanran::UpdateSyncHS()
{

	syncHS_timestamp.start_count = htim12.Instance->CNT;



	uint8_t userButton0 = HAL_GPIO_ReadPin(USER_SW0_GPIO_Port, USER_SW0_Pin);
	uint8_t userButton1 = HAL_GPIO_ReadPin(USER_SW1_GPIO_Port, USER_SW1_Pin);

	if(userButton0 == 0 && userButton0_prev == 1)
	{
		kicker.kickStraight(1);
	}
	if(userButton1 == 0 && userButton1_prev == 1)
	{
		kicker.kickChip(1);
	}
	userButton0_prev = userButton0;
	userButton1_prev = userButton1;

	kicker.update();


#if 0
	omniCmd.world_vel_x = simulink.m_data[0];
	omniCmd.world_vel_y = simulink.m_data[1];
	omniCmd.omega = simulink.m_data[2];
#endif


	timeElapsed_hs_count += 1;

	omni.update(&omniCmd);

	odo_x = omni.m_robotState.world_x;
	odo_y = omni.m_robotState.world_y;
	odo_theta = omni.m_robotState.world_theta;

	for(int ch = 0; ch < 4; ch++) wheel_theta[ch] = canMotorIF.motor[ch].get_theta();


	Iq_ref = canMotorIF.motor[0].get_Iq_ref();


	dump_update();


	syncHS_timestamp.end_count = htim12.Instance->CNT;

}

/**
 * @fn void UpdateSyncLS()
 * @brief 表示系用低速ループ
 *
 */
void Sanran::UpdateSyncLS()
{

	syncLS_timestamp.start_count = htim13.Instance->CNT;

	power.update();

	deg += 0.01;
	if(deg > 1.0) deg -= 1.0;

	onBrdLED.setHSV(deg, 1.0, 1.0);

	// bno055.updateIMU();
	ballSensor.update();


//	if(ballSensor.read() > 0.15) dribbler.setSlow();
//	else dribbler.setFast();

	matcha.Update();

	if(matcha.newDataAvailable() && matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_NORMAL)
	{

		omniCmd.world_x = matcha.normal_cmd.cmd_x;
		omniCmd.world_y = matcha.normal_cmd.cmd_y;
		omniCmd.world_theta = matcha.normal_cmd.cmd_theta;
		omniCmd.world_vel_x = matcha.normal_cmd.cmd_vx;
		omniCmd.world_vel_y = matcha.normal_cmd.cmd_vy;
		omniCmd.world_omega = matcha.normal_cmd.cmd_omega;
		omniCmd.vel_limit = matcha.normal_cmd.vel_limit;

		if(matcha.normal_cmd.kick)
		{
			if(matcha.normal_cmd.chip)
			{
				kicker.kickChip(matcha.normal_cmd.kickPower);
			}
			else
			{
				kicker.kickStraight(matcha.normal_cmd.kickPower);
			}
		}

		if(matcha.normal_cmd.dribble)
		{
			dribbler.setPower(matcha.normal_cmd.dribblePower);
		}
		else
		{
			dribbler.setPower(0);
		}

		if(matcha.normal_cmd.vision_error == false)
		{
			omni.correctPosition(matcha.normal_cmd.fb_x, matcha.normal_cmd.fb_y, matcha.normal_cmd.fb_theta);
		}

		omni.setControlType(OmniWheel::TYPE_WORLD_POSITION);

	}
	else if(matcha.newDataAvailable() && matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_MANUAL)
	{

		omniCmd.robot_vel_x = 0.0f;
		omniCmd.robot_vel_y = 0.0f;
		omniCmd.robot_omega = 0.0f;

//
//		omniCmd.robot_vel_x = matcha.manual_cmd.cmd_vx;
//		omniCmd.robot_vel_y = matcha.manual_cmd.cmd_vy;
//		omniCmd.robot_omega = matcha.manual_cmd.cmd_omega;
//
//		if(matcha.manual_cmd.kick)
//		{
//			if(matcha.manual_cmd.chip)
//			{
//				kicker.kickChip(matcha.manual_cmd.kickPower);
//			}
//			else
//			{
//				kicker.kickStraight(matcha.manual_cmd.kickPower);
//			}
//		}
//
//		if(matcha.manual_cmd.dribble)
//		{
//			dribbler.setPower(matcha.manual_cmd.dribblePower);
//		}
//		else
//		{
//			dribbler.setPower(0);
//		}

		omni.setControlType(OmniWheel::TYPE_ROBOT_P_DOB);

	}
	else if(matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_TIMEOUT)
	{

		omniCmd.vel_limit = 0.0f;

		omni.setControlType(OmniWheel::TYPE_WORLD_POSITION);

	}

	syncLS_timestamp.end_count = htim13.Instance->CNT;

}


void Sanran::CAN_Rx_Callback(FDCAN_HandleTypeDef *hfdcan)
{

	canMotorIF.update_CAN_Rx();

}



void Sanran::UART_Rx_Callback(UART_HandleTypeDef *huart)
{

	//simulink.dataReceivedCallback(huart);

	//matcha.dataReceivedCallback(huart);

}



void Sanran::dump_update()
{

	dump.setValue( 0, timeElapsed_hs_count * 0.001f);

	dump.setValue( 1, canMotorIF.motor[0].get_Iq_ref() );
	dump.setValue( 2, canMotorIF.motor[0].get_Iq() );
	dump.setValue( 3, canMotorIF.motor[0].get_omega() );
	dump.setValue( 4, canMotorIF.motor[0].get_theta() );

	dump.setValue( 5, canMotorIF.motor[1].get_Iq_ref() );
	dump.setValue( 6, canMotorIF.motor[1].get_Iq() );
	dump.setValue( 7, canMotorIF.motor[1].get_omega() );
	dump.setValue( 8, canMotorIF.motor[1].get_theta() );

	dump.setValue( 9, canMotorIF.motor[2].get_Iq_ref() );
	dump.setValue(10, canMotorIF.motor[2].get_Iq() );
	dump.setValue(11, canMotorIF.motor[2].get_omega() );
	dump.setValue(12, canMotorIF.motor[2].get_theta() );

	dump.setValue(13, canMotorIF.motor[3].get_Iq_ref() );
	dump.setValue(14, canMotorIF.motor[3].get_Iq() );
	dump.setValue(15, canMotorIF.motor[3].get_omega() );
	dump.setValue(16, canMotorIF.motor[3].get_theta() );

	dump.setValue(17, omni.m_cmd.omega_w[0]);
	dump.setValue(18, omni.m_cmd.omega_w[1]);
	dump.setValue(19, omni.m_cmd.omega_w[2]);
	dump.setValue(20, omni.m_cmd.omega_w[3]);

	dump.setValue(21, omni.m_cmd.robot_vel_x);
	dump.setValue(22, omni.m_cmd.robot_vel_y);
	dump.setValue(23, omni.m_cmd.robot_omega);

	dump.setValue(24, omni.m_cmd.world_x);
	dump.setValue(25, omni.m_cmd.world_y);
	dump.setValue(26, omni.m_cmd.world_theta);

	dump.setValue(27, omni.m_cmd.world_vel_x);
	dump.setValue(28, omni.m_cmd.world_vel_y);
	dump.setValue(29, omni.m_cmd.world_omega);

	dump.setValue(30, omni.m_robotState.world_x);
	dump.setValue(31, omni.m_robotState.world_y);
	dump.setValue(32, omni.m_robotState.world_theta);

	dump.setValue(33, matcha.normal_cmd.fb_x);
	dump.setValue(34, matcha.normal_cmd.fb_y);
	dump.setValue(35, matcha.normal_cmd.fb_theta);
	dump.setValue(36, matcha.normal_cmd.fb_timestamp * 1E-3);

	dump.send();


}



