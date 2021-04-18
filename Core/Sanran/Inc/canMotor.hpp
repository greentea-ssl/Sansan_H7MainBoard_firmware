

#ifndef _CAN_MOTOR_HPP_
#define _CAN_MOTOR_HPP_


#include "stm32h7xx_hal.h"

#include "main.h"



class CanMotor
{

public:

	CanMotor(uint32_t MotorID);


	void resUpdate(int16_t Iq_int16, int16_t omega_int16, uint16_t theta_uint16);

	bool resIsUpdated(){ return m_resUpdated; }

	void clearUpdateFlag(){ m_resUpdated = 0; }

	void set_Iq_ref(float Iq_ref){ m_Iq_ref = Iq_ref;}

	uint32_t getRtnID(){ return 0x400 + m_motorID; }

	uint32_t get_motorID(){ return m_motorID; }



	float get_Iq_ref(){ return m_Iq_ref; }

	float get_Iq(){ return m_Iq; }

	float get_omega(){ return m_omega; }

	float get_theta(){ return m_theta; }



private:

	uint32_t m_motorID;

	int16_t m_Iq_int16;
	int16_t m_omega_int16;
	uint16_t m_theta_uint16;

	float m_Iq_ref;
	float m_Iq;
	float m_omega;
	float m_theta;

	bool m_resUpdated;

};



class CanMotorIF
{

public:

	CanMotorIF(FDCAN_HandleTypeDef *hfdcan);

	void update_CAN_Rx();

	void send_Iq_ref();

	void setCurrent(float I0, float I1, float I2, float I3);


	CanMotor motor[4];


private:

	FDCAN_HandleTypeDef *m_hfdcan;


};






#endif /* _CAN_MOTOR_HPP_ */
