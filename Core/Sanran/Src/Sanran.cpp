


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
extern SPI_HandleTypeDef hspi4;


/* Debug variables */

volatile float omega_w = 0.0;
volatile float omega_w_ref = 0.0;
volatile float Iq_ref = 0.0;
volatile float lpf_test = 0.0f;

volatile float odo_x = 0.0f;
volatile float odo_y = 0.0f;
volatile float odo_theta = 0.0f;

volatile float wheel_theta[4] = {0};


#define DISP_SETUP_RESULT(FUNC, NAME, IS_CRITICAL)	do{ \
														printf("\n Setting %s ... \n", NAME); \
														if(FUNC()){printf("\t\t\t\t[OK]\n"); \
														}else{ \
															printf("\t\t\t\t[ERROR]\n"); \
															if(IS_CRITICAL){Error_Handler();}} \
													}while(0);


void captureBall(BallInformation* ball_data, OmniWheel::Cmd_t* omniCmd, OmniWheel* omni){
  omni->setControlType(OmniWheel::TYPE_ROBOT_P_DOB);

  float gain_w = 0.2;
  float gain_x = 0.03;
  float gain_y = 0.02;
  if(ball_data->status == 0){
    omniCmd->robot_vel_x = 0;
    omniCmd->robot_vel_y = 0;
    omniCmd->robot_omega = 0;
    return;
  }
  omniCmd->robot_vel_x = fminf(0.8, fmaxf(-0.5 , gain_x * ball_data->x));
  omniCmd->robot_vel_y = 0;//fminf(0.5, fmaxf(-0.5 , gain_y * ball_data->y));
  omniCmd->robot_omega = 0;//fminf(10, fmaxf(-10 , -gain_w * ball_data->x));
//  printf("omniCmd.robot_vel_x: %f,   ", omniCmd->robot_vel_x);
//  printf("omniCmd.robot_vel_y: %f,   ", omniCmd->robot_vel_y);
//  printf("omniCmd.robot_omega: %f,   \n\r", omniCmd->robot_omega);
//

}


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
	  matcha(&huart5, 0.2, 5.0, 0.01),
	  dump(&hspi4),
	  ball_info_communication(&huart1)
{

	memset(&omniCmd, 0x00, sizeof(omniCmd));

}



void Sanran::setup()
{

	uint8_t userButton0 = HAL_GPIO_ReadPin(USER_SW0_GPIO_Port, USER_SW0_Pin);
	uint8_t userButton1 = HAL_GPIO_ReadPin(USER_SW1_GPIO_Port, USER_SW1_Pin);

	printf("\nHello. ");

	// ボタンは負論理
	if(1)
	{
		opeMode = OPE_MODE_DEBUG;
		printf("[DEBUG_MODE]\n\n");
		buzzer.play(Buzzer::SOUND_STARTUP_DEBUG);
	}
	else if(userButton1 == 0)
	{
		opeMode = OPE_MODE_MANUAL;
		printf("[MANUAL_MODE]\n\n");
		buzzer.play(Buzzer::SOUND_STARTUP_MANUAL);
	}
	else
	{
		opeMode = OPE_MODE_NORMAL;
		printf("[NORMAL_MODE]\n\n");
		buzzer.play(Buzzer::SOUND_STARTUP_NORMAL);
	}

	power.enableSupply();

	delay_ms(200);

	printf("********** Initialize ********************\n\n");

	printf("\n Setting %s ... \n", "On Board LED");
	DISP_SETUP_RESULT(onBrdLED.setup, "On Board LED", false);

	DISP_SETUP_RESULT(canMotorIF.setup, "CAN motor IF", true);

	DISP_SETUP_RESULT(bno055.setup, "BNO055", false);

	DISP_SETUP_RESULT(omni.setup, "Omni Wheel", (opeMode==OPE_MODE_NORMAL));

	DISP_SETUP_RESULT(kicker.setup, "Kicker", true);

	DISP_SETUP_RESULT(matcha.setup, "Matcha Serial", true);

	timeElapsed_hs_count = 0;

	onBrdLED.setRGB(0, 0, 0);

	deg = 0.0;

	userButton0_prev = 1;
	userButton1_prev = 1;

	dribbler.setup();


	buzzer.play(Buzzer::SOUND_NOTIFY);

	// WachDog(software reset) initialize
	watchdog_CAN_count = 0;
	watchdog_CAN_threshold = 1000;
	watchdog_UART_count = 0;
	watchdog_UART_threshold = 10000;

	//setup communication with raspberry pi
	ball_info_communication.init();

	if(opeMode == OPE_MODE_NORMAL)
	{
		watchdog_enable = true;
	}
	else
	{
		watchdog_enable = false;
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

	kicker.update();

	omni.update(&omniCmd);

	odo_x = omni.m_robotState.world_x;
	odo_y = omni.m_robotState.world_y;
	odo_theta = omni.m_robotState.world_theta;
	for(int ch = 0; ch < 4; ch++)
	{
		wheel_theta[ch] = canMotorIF.motor[ch].get_theta();
	}

	// Update WatchDog
	update_watchdog();

	update_dump();

	timeElapsed_hs_count += 1;
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

	// get ball information
	BallInformationResult result = ball_info_communication.ReceiveBallInformation(&rx_ball);

  switch(result){
  case BALLINFO_TIMEOUT:
	  rx_ball.x = 0;
	  rx_ball.y = 0;
	  rx_ball.vx = 0;
	  rx_ball.vy = 0;
	  rx_ball.status = 0;
	  // go to the next case since no break!!!!!!!!!!!!!!!
  case BALLINFO_SUCCESS:
//    printf("decoded:");
//    printf("x:%f, y:%f, status:%ld\n\r", rx_ball.x, rx_ball.y, rx_ball.status);
//    printf("\n\r");
    captureBall(&rx_ball, &omniCmd, &omni);
    break;
  case BALLINFO_DECODE_NULL_POINTER:
    printf("BALLINFO_DECODE_NULL_POINTER\n\r");
    break;
  case BALLINFO_DECODE_OUT_BUFFER_OVERFLOW:
    printf("BALLINFO_DECODE_OUT_BUFFER_OVERFLOW\n\r");
    break;
  case BALLINFO_DECODE_ZERO_BYTE_IN_INPUT:
    printf("BALLINFO_DECODE_ZERO_BYTE_IN_INPUT\n\r");
    break;
  case BALLINFO_DECODE_INPUT_TOO_SHORT:
    printf("BALLINFO_DECODE_INPUT_TOO_SHORT\n\r");
  case BALLINFO_DECODE_UNKNOWN:
    printf("BALLINFO_DECODE_UNKNOWN\n\r");
    break;
  case BALLINFO_FRAME_FAIL:
//      printf("BALLINFO_FRAME_FAIL\n\r");
    break;
  case BALLINFO_FRAME_INVALID_LENGTH:
    printf("BALLINFO_FRAME_INVALID_LENGTH\n\r");
    break;
  default:
    printf("unknown error\n\r");
	  break;
  }
	// decide robot speed

//	if(matcha.newDataAvailable() && matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_NORMAL)
//	{
//		omniCmd.world_x = matcha.normal_cmd.cmd_x;
//		omniCmd.world_y = matcha.normal_cmd.cmd_y;
//		omniCmd.world_theta = matcha.normal_cmd.cmd_theta;
//		omniCmd.world_vel_x = matcha.normal_cmd.cmd_vx;
//		omniCmd.world_vel_y = matcha.normal_cmd.cmd_vy;
//		omniCmd.world_omega = matcha.normal_cmd.cmd_omega;
//		omniCmd.vel_limit = matcha.normal_cmd.vel_limit;
//		omniCmd.omega_limit = matcha.normal_cmd.omega_limit;
//
//		if(matcha.normal_cmd.kick)
//		{
//			if(matcha.normal_cmd.chip)
//			{
//				kicker.kickChip(matcha.normal_cmd.kickPower);
//			}
//			else
//			{
//				kicker.kickStraight(matcha.normal_cmd.kickPower);
//			}
//		}
//
//		if(matcha.normal_cmd.dribble)
//		{
//			dribbler.setPower(matcha.normal_cmd.dribblePower);
//		}
//		else
//		{
//			dribbler.setPower(0);
//		}
//
//		if(matcha.normal_cmd.vision_error == false)
//		{
//			omni.correctPosition(matcha.normal_cmd.fb_x, matcha.normal_cmd.fb_y, matcha.normal_cmd.fb_theta);
//		}
//
//		omni.setControlType(OmniWheel::TYPE_WORLD_POSITION);
//
//	}
//	else if(matcha.newDataAvailable() && matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_MANUAL)
//	{
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
//
//		omni.setControlType(OmniWheel::TYPE_ROBOT_P_DOB);
//
//	}
//	else if(matcha.getReceiveState() == MatchaSerial::RECEIVE_STATE_TIMEOUT)
//	{
//
//		omniCmd.vel_limit = 0.0f;
//
//		omni.setControlType(OmniWheel::TYPE_WORLD_POSITION);
//
//	}

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


bool Sanran::display_result(bool(*setup_func)(), char* component_name)
{
	printf("\n Setting %s ... \n", component_name);
	bool boolStatus = setup_func();
	if(boolStatus){
		printf("\t\t\t\t[OK]\n");
	}else{
		printf("\t\t\t\t[ERROR]\n");
	}
	return boolStatus;
}


void Sanran::update_watchdog()
{
	if(watchdog_enable)
	{
		if(watchdog_CAN_count >= watchdog_CAN_threshold)
		{
			power.disableSupply();
			HAL_NVIC_SystemReset();
		}
		if(watchdog_UART_count >= watchdog_UART_threshold)
		{
			power.disableSupply();
			HAL_NVIC_SystemReset();
		}
		watchdog_CAN_count++;
		watchdog_UART_count++;
	}
	else
	{
		watchdog_CAN_count = 0;
		watchdog_UART_count = 0;
	}

	if(omni.get_last_error_status() == OmniWheel::ERROR_NONE)
	{
		watchdog_CAN_count = 0;
	}
	if(matcha.getReceiveState() != MatchaSerial::RECEIVE_STATE_TIMEOUT)
	{
		watchdog_UART_count = 0;
	}
}


void Sanran::update_dump()
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


void Sanran::Error_Handler()
{
	buzzer.setNoteNumber(76+12);
	while(1){
		buzzer.on();
		delay_ms(100);
		buzzer.off();
		delay_ms(100);
	}

}



