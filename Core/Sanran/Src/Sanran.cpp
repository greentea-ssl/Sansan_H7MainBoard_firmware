


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
	boolStatus = matcha.setup();
	matcha.settingTimeout(0.1, 0.01);
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}

	timeElapsed_hs_count = 0;

	count = 0;

	onBrdLED.setRGB(0, 0, 0);


	deg = 0.0;

	userButton0_prev = 1;
	userButton1_prev = 1;

	power.enableSupply();


	dribbler.setup();
	//dribbler.setStop();
	//dribbler.setFast();
	dribbler.setSlow();


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


	HAL_Delay(100);

	Sync_loop_timestamp_t LS_timestamp;
	LS_timestamp = syncLS_timestamp;

	Sync_loop_timestamp_t HS_timestamp;
	HS_timestamp = syncHS_timestamp;

	//printf("start:%d,\t end:%d,\t period:%d\r\n", LS_timestamp.start_count, LS_timestamp.end_count, htim13.Init.Period);

	printf("start:%d,\t end:%d,\t period:%d\r\n", HS_timestamp.start_count, HS_timestamp.end_count, htim12.Init.Period);



	/*
	uint8_t st, er;
	bno055.read(0x39, &st);
	bno055.read(0x3D, &er);

	printf("Roll  = %f rad.\n", bno055.get_IMU_roll());
	printf("Pitch = %f rad.\n", bno055.get_IMU_pitch());
	printf("Yaw   = %f rad.\n", bno055.get_IMU_yaw());
	printf("status = %d, 0x%02x, 0x%02x\n", bno055.getStatus(), st, er);
	printf("\e[4A");
	*/


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


	if(this->omni.get_controlType() == OmniWheel::TYPE_WORLD_P_DOB)
	{
		omniCmd.world_vel_x = matcha.cmd.cmd_vx;
		omniCmd.world_vel_y = matcha.cmd.cmd_vy;
		omniCmd.world_omega = matcha.cmd.cmd_omega;
	}
	else if(this->omni.get_controlType() == OmniWheel::TYPE_ROBOT_P_DOB)
	{
		omniCmd.robot_vel_x = matcha.cmd.cmd_vx;
		omniCmd.robot_vel_y = matcha.cmd.cmd_vy;
		omniCmd.robot_omega = matcha.cmd.cmd_omega;
	}


	omega_w = canMotorIF.motor[0].get_omega();


	timeElapsed_hs_count += 1;

	count += 1;


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

	//printf("ball : %f\n", ballSensor.read());

	if(ballSensor.read() > 0.15) dribbler.setSlow();
	else dribbler.setFast();

	//omni.correctAngle(2*M_PI - bno055.get_IMU_yaw());


	//printf("USER_SW0 = %f\n", omega_w_ref);





	//printf("%f, %f, %f\n", simulink.m_data[0], simulink.m_data[1], simulink.m_data[2]);

	matcha.Update();
	if(matcha.getTimeoutState() == MatchaSerial::TIMEOUT_NONE)
	{
		omni.setControlType(OmniWheel::TYPE_WORLD_POSITION);

		if(this->omni.get_controlType() == OmniWheel::TYPE_WORLD_POSITION)
		{
			omniCmd.world_x = matcha.cmd.cmd_x;
			omniCmd.world_y = matcha.cmd.cmd_y;
			omniCmd.world_theta = matcha.cmd.cmd_theta;
			omniCmd.world_vel_x = matcha.cmd.cmd_vx;
			omniCmd.world_vel_y = matcha.cmd.cmd_vy;
			omniCmd.world_omega = matcha.cmd.cmd_omega;
		}
		if(matcha.cmd.vision_error == false)
		{
			//omni.correctAngle(matcha.cmd.fb_theta);
			omni.correctPosition(matcha.cmd.fb_x, matcha.cmd.fb_y, matcha.cmd.fb_theta - M_PI*0.5f);
		}
	}
	else
	{
		omniCmd.omega_w[0] = 0.0f;
		omniCmd.omega_w[1] = 0.0f;
		omniCmd.omega_w[2] = 0.0f;
		omniCmd.omega_w[3] = 0.0f;
		omni.setControlType(OmniWheel::TYPE_INDEP_P_DOB);
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

	dump.send();


}



