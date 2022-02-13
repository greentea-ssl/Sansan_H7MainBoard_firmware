


#include "Sanran.hpp"



#include <stdio.h>
#include "main.h"


#include "stm32h753xx.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern FDCAN_HandleTypeDef hfdcan2;
extern I2C_HandleTypeDef hi2c2;
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
	  kicker(0.01, 0.5),
	  omni(OmniWheel::TYPE_WORLD_POSITION, &canMotorIF),
	  matcha(&huart5)
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

	/***** Setting Matcha Serial *****/
	printf("\n Setting Matcha Serial ... \n");
	boolStatus = matcha.setup();
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}


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


#if 0
	omniCmd.world_vel_x = simulink.m_data[0];
	omniCmd.world_vel_y = simulink.m_data[1];
	omniCmd.omega = simulink.m_data[2];
#endif


	if(this->omni.get_controlType() == OmniWheel::TYPE_WORLD_P_DOB)
	{
		omniCmd.world_vel_x = matcha.cmd.cmd_vx;
		omniCmd.world_vel_y = matcha.cmd.cmd_vy;
		omniCmd.omega = matcha.cmd.cmd_omega;
	}
	else if(this->omni.get_controlType() == OmniWheel::TYPE_ROBOT_P_DOB)
	{
		omniCmd.robot_vel_x = matcha.cmd.cmd_vx;
		omniCmd.robot_vel_y = matcha.cmd.cmd_vy;
		omniCmd.omega = matcha.cmd.cmd_omega;
	}


	omega_w = canMotorIF.motor[0].get_omega();

	count += 1;


	omni.update(&omniCmd);

	odo_x = omni.m_robotState.world_x;
	odo_y = omni.m_robotState.world_y;
	odo_theta = omni.m_robotState.world_theta;

	for(int ch = 0; ch < 4; ch++) wheel_theta[ch] = canMotorIF.motor[ch].get_theta();


	Iq_ref = canMotorIF.motor[0].get_Iq_ref();



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

	bno055.updateIMU();
	ballSensor.update();

	//printf("ball : %f\n", ballSensor.read());

	if(ballSensor.read() > 0.15) dribbler.setSlow();
	else dribbler.setFast();

	//omni.correctAngle(2*M_PI - bno055.get_IMU_yaw());


	//printf("USER_SW0 = %f\n", omega_w_ref);

	uint8_t userButton0 = HAL_GPIO_ReadPin(USER_SW0_GPIO_Port, USER_SW0_Pin);
	uint8_t userButton1 = HAL_GPIO_ReadPin(USER_SW1_GPIO_Port, USER_SW1_Pin);

	if(userButton0 == 0 && userButton1_prev == 1)
	{
		kicker.kickStraight();
	}
	if(userButton1 == 0 && userButton1_prev == 1)
	{
		kicker.kickChip();
	}
	userButton0_prev = userButton0;
	userButton1_prev = userButton1;


	kicker.update();

	//printf("%f, %f, %f\n", simulink.m_data[0], simulink.m_data[1], simulink.m_data[2]);

	if(matcha.Update())
	{
		if(matcha.cmd.robot_ID == 0x0F)
		if(this->omni.get_controlType() == OmniWheel::TYPE_WORLD_POSITION)
		{
			omniCmd.world_x = matcha.cmd.cmd_x;
			omniCmd.world_y = matcha.cmd.cmd_y;
			omniCmd.world_theta = matcha.cmd.cmd_theta;
		}
		if(matcha.cmd.vision_error == false)
		{
			//omni.correctAngle(matcha.cmd.fb_theta);
			omni.correctPosition(matcha.cmd.fb_x, matcha.cmd.fb_y, matcha.cmd.fb_theta);
		}
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





