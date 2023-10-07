


#include "canMotor.hpp"

#include "stdio.h"


CanMotor::CanMotor(uint32_t motorID) : m_motorID(motorID), m_resUpdated(0)
{
}


void CanMotor::resUpdate(int16_t Iq_int16, int16_t omega_int16, uint16_t theta_uint16)
{

	m_Iq_int16 = Iq_int16;
	m_omega_int16 = omega_int16;
	m_theta_uint16 = theta_uint16;

	m_Iq = m_Iq_int16 * 0.0009765625f; // 1/1024
	m_theta = m_theta_uint16 * 0.0003834951969714103f; // 2*pi/16384
	m_omega = m_omega_int16 * 0.03125f; // 1/32

	m_resUpdated = 1;

}

void CanMotor::paramUpdate(uint16_t Kv, uint16_t Irated_uint16)
{

	m_Kv = Kv;
	m_Irated_uint16 = Irated_uint16;

	m_Irated = m_Irated_uint16 / 1024.0f;

	m_resUpdated = 1;

}


CanMotorIF::CanMotorIF(FDCAN_HandleTypeDef *hfdcan) : motor{0, 1, 2, 3}, m_hfdcan(hfdcan)
{




}


/**
 * @fn void setup()
 * @brief CanMotorIF peripheral setting
 *
 */
bool CanMotorIF::setup()
{

	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x00;//0x400;
	sFilterConfig.FilterID2 = 0x000;//0x7fc; // 0b0111 1111 1100
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.RxBufferIndex = 0;
	sFilterConfig.IsCalibrationMsg = 0;


	if(HAL_FDCAN_ConfigFilter(m_hfdcan, &sFilterConfig) != HAL_OK)
	{
		return false;
		//Error_Handler();
	}

	if(HAL_FDCAN_ConfigGlobalFilter(m_hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, ENABLE, ENABLE) != HAL_OK)
	{
		return false;
		//Error_Handler();
	}

	if(HAL_FDCAN_ActivateNotification(m_hfdcan, FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		return false;
		//Error_Handler();
	}

	if(HAL_FDCAN_Start(m_hfdcan) != HAL_OK)
	{
		return false;
		//Error_Handler();
	}

	return true;
}



void CanMotorIF::update_CAN_Rx()
{

	FDCAN_RxHeaderTypeDef canRxHeader;
	uint8_t canRxData[8];
	uint8_t motor_channel;

	HAL_FDCAN_GetRxMessage(m_hfdcan, FDCAN_RX_FIFO0, &canRxHeader, canRxData);

	if((canRxHeader.Identifier & 0xFFFFFFFC) == 0x400
			&& canRxHeader.DataLength == FDCAN_DLC_BYTES_8)
	{
		motor_channel = canRxHeader.Identifier & 0x03;

		int16_t Iq_int16 = ((int16_t)canRxData[2] << 8) | canRxData[1];
		int16_t omega_int16 = ((int16_t)canRxData[6] << 8) | canRxData[5];
		uint16_t theta_uint16 = ((uint16_t)canRxData[4] << 8) | canRxData[3];

		motor[motor_channel].resUpdate(Iq_int16, omega_int16, theta_uint16);
	}
	else if((canRxHeader.Identifier & 0xFFFFFFFC) == 0x410
			&& canRxHeader.DataLength == FDCAN_DLC_BYTES_4)
	{
		motor_channel = canRxHeader.Identifier & 0x03;

		uint16_t Kv = ((int16_t)canRxData[1] << 8) | canRxData[0];
		uint16_t Irated_uint16 = ((uint16_t)canRxData[3] << 8) | canRxData[2];

		motor[motor_channel].paramUpdate(Kv, Irated_uint16);
	}


}


bool CanMotorIF::all_response_arrived()
{
	const int MOTOR_NUM = 4;
	for(int i = 0; i < MOTOR_NUM; i++)
	{
		if(!motor[i].resIsUpdated())
		{
			return false;
		}
	}
	return true;
}

bool CanMotorIF::read_motor_param()
{
	const int retry_time_ms = 200;
	const int retry_timeout = 5;

	for(int retry_count = 0; retry_count < retry_timeout; retry_count++)
	{
		for(int i = 0; i < 4; i++)
		{
			motor[i].clearUpdateFlag();
		}
		send_request_param();
		for(int i = 0; i < retry_time_ms / 10; i++)
		{
			HAL_Delay(10);
			if(all_response_arrived() == true)
			{
				return true; // Successful
			}
		}
	}
	return false; // Timeout
}


void CanMotorIF::send_request_param()
{

	FDCAN_TxHeaderTypeDef fdcanTxHeader;
	uint8_t fdcanTxData[8];

	fdcanTxHeader.Identifier = 0x310;
	fdcanTxHeader.IdType = FDCAN_STANDARD_ID;
	fdcanTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	fdcanTxHeader.DataLength = FDCAN_DLC_BYTES_0;
	fdcanTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	fdcanTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	fdcanTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	fdcanTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	fdcanTxHeader.MessageMarker = 0;

	//HAL_FDCAN_ActivateNotification(&hfdcan2, 0, FDCAN_IT_TX_COMPLETE);

	HAL_FDCAN_AddMessageToTxFifoQ(m_hfdcan, &fdcanTxHeader, fdcanTxData);


}


void CanMotorIF::send_Iq_ref()
{

	FDCAN_TxHeaderTypeDef fdcanTxHeader;
	uint8_t fdcanTxData[8];

	int16_t Iq_ref_int[4];


	fdcanTxHeader.Identifier = 0x300;
	fdcanTxHeader.IdType = FDCAN_STANDARD_ID;
	fdcanTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	fdcanTxHeader.DataLength = FDCAN_DLC_BYTES_8;
	fdcanTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	fdcanTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	fdcanTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	fdcanTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	fdcanTxHeader.MessageMarker = 0;

	Iq_ref_int[0] = (int16_t)(motor[0].get_Iq_ref() * 1024);
	Iq_ref_int[1] = (int16_t)(motor[1].get_Iq_ref() * 1024);
	Iq_ref_int[2] = (int16_t)(motor[2].get_Iq_ref() * 1024);
	Iq_ref_int[3] = (int16_t)(motor[3].get_Iq_ref() * 1024);

	fdcanTxData[0] = Iq_ref_int[0] & 0xff;
	fdcanTxData[1] = (Iq_ref_int[0] >> 8) & 0xff;

	fdcanTxData[2] = Iq_ref_int[1] & 0xff;
	fdcanTxData[3] = (Iq_ref_int[1] >> 8) & 0xff;

	fdcanTxData[4] = Iq_ref_int[2] & 0xff;
	fdcanTxData[5] = (Iq_ref_int[2] >> 8) & 0xff;

	fdcanTxData[6] = Iq_ref_int[3] & 0xff;
	fdcanTxData[7] = (Iq_ref_int[3] >> 8) & 0xff;


	//HAL_FDCAN_ActivateNotification(&hfdcan2, 0, FDCAN_IT_TX_COMPLETE);

	HAL_FDCAN_AddMessageToTxFifoQ(m_hfdcan, &fdcanTxHeader, fdcanTxData);


}



void CanMotorIF::setCurrent(float I0, float I1, float I2, float I3)
{

	motor[0].set_Iq_ref(I0);
	motor[1].set_Iq_ref(I1);
	motor[2].set_Iq_ref(I2);
	motor[3].set_Iq_ref(I3);

}




