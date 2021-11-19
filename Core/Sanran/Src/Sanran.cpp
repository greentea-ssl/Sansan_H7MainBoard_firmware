


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
	  omni(OmniWheel::TYPE_WORLD_P_DOB, &canMotorIF),
	  matcha(&huart5)
{


}



void Sanran::setup()
{

	/** Peripheral setting **/

	onBrdLED.setup();

	canMotorIF.setup();





	/***********************/


	printf("oppai...\n");

	count = 0;

	onBrdLED.setRGB(0, 0, 0);

	buzzer.sound_startup();

	//bno055.write(0x3D, 0x08);

	printf("BNO055, initialize result: %d\n", bno055.getStatus());

	printf("BNO055, who am i check : %d\n", bno055.checkChipID());

	deg = 0.0;

	userButton0_prev = 1;
	userButton1_prev = 1;

	power.enableSupply();


	dribbler.setup();
	//dribbler.setStop();
	//dribbler.setFast();
	dribbler.setSlow();


	delay_ms(4000); // wait for BLDC sensor calibration


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

	power.update();

	HAL_Delay(1);

	deg += 0.01;
	if(deg > 1.0) deg -= 1.0;

	onBrdLED.setHSV(deg, 1.0, 1.0);

	bno055.updateIMU();

	uint8_t st, er;
	bno055.read(0x39, &st);
	bno055.read(0x3D, &er);

	/*
	printf("Roll  = %f rad.\n", bno055.get_IMU_roll());
	printf("Pitch = %f rad.\n", bno055.get_IMU_pitch());
	printf("Yaw   = %f rad.\n", bno055.get_IMU_yaw());
	printf("status = %d, 0x%02x, 0x%02x\n", bno055.getStatus(), st, er);
	printf("\e[4A");
	*/

	ballSensor.update();

	//printf("ball : %f\n", ballSensor.read());

	if(ballSensor.read() > 0.15) dribbler.setSlow();
	else dribbler.setFast();


	omni.correctAngle(2*M_PI - bno055.get_IMU_yaw());


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

	matcha.Update();




}

/**
 * @fn void UpdateSyncHS()
 * @brief 制御用高速ループ
 *
 */
void Sanran::UpdateSyncHS()
{


#if 0
	omniCmd.world_vel_x = simulink.m_data[0];
	omniCmd.world_vel_y = simulink.m_data[1];
	omniCmd.omega = simulink.m_data[2];
#endif


	omniCmd.world_vel_x = matcha.cmd.vel_x;
	omniCmd.world_vel_y = matcha.cmd.vel_y;
	omniCmd.omega = matcha.cmd.omega;


	omega_w = canMotorIF.motor[0].get_omega();

	count += 1;


	omni.update(&omniCmd);

	odo_x = omni.m_robotState.world_x;
	odo_y = omni.m_robotState.world_y;
	odo_theta = omni.m_robotState.world_theta;

	for(int ch = 0; ch < 4; ch++) wheel_theta[ch] = canMotorIF.motor[ch].get_theta();


	Iq_ref = canMotorIF.motor[0].get_Iq_ref();

}

/**
 * @fn void UpdateSyncLS()
 * @brief 表示系用低速ループ
 *
 */
void Sanran::UpdateSyncLS()
{

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





