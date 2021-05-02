

#include "OmniWheel.hpp"

#include <math.h>


volatile float estTorque = 0.0f;


OmniWheel::OmniWheel(ControlType_t type, CanMotorIF *canMotorIF, Param_t *param) : m_type(type), m_canMotorIF(canMotorIF), m_param(*param)
{

	for(int ch = 0; ch < 4; ch++)
	{
		m_convMat_robot2motor[ch][0] = cosf(m_param.wheel_pos_theta_deg[ch] * M_PI / 180.0f) / m_param.wheel_r[ch];
		m_convMat_robot2motor[ch][1] = sinf(m_param.wheel_pos_theta_deg[ch] * M_PI / 180.0f) / m_param.wheel_r[ch];
		m_convMat_robot2motor[ch][2] = -m_param.wheel_pos_r[ch] / m_param.wheel_r[ch];
	}

	for(int i = 0; i < 4; i++)
		dob[i].setParam(m_param.Ktn, m_param.Jmn, m_param.g_dis, m_param.Ts);


	for(int ch = 0; ch < 4; ch++)
		m_canMotorIF->motor[ch].set_Iq_ref(0.0f);

}

OmniWheel::OmniWheel(ControlType_t type, CanMotorIF *canMotorIF) :
		m_type(type),
		m_canMotorIF(canMotorIF)
{

	m_param.Jmn = 5.2E-5;
	m_param.Kp = 0.1;
	m_param.Ktn = (60.0f / (320 * 2 * M_PI));
	m_param.Ts = 1E-3;
	m_param.g_dis = 200;

	for(int i = 0; i < 4; i++)
	{
		m_param.wheel_pos_r[i] = 74.7E-3;
		m_param.wheel_r[i] = 27.427E-3;
	}

	m_param.wheel_pos_theta_deg[0] = 60;
	m_param.wheel_pos_theta_deg[1] = 135;
	m_param.wheel_pos_theta_deg[2] = -135;
	m_param.wheel_pos_theta_deg[3] = -60;

	for(int ch = 0; ch < 4; ch++)
	{
		m_convMat_robot2motor[ch][0] = cosf(m_param.wheel_pos_theta_deg[ch] * M_PI / 180.0f) / m_param.wheel_r[ch];
		m_convMat_robot2motor[ch][1] = sinf(m_param.wheel_pos_theta_deg[ch] * M_PI / 180.0f) / m_param.wheel_r[ch];
		m_convMat_robot2motor[ch][2] = -m_param.wheel_pos_r[ch] / m_param.wheel_r[ch];
	}


	for(int i = 0; i < 4; i++)
		dob[i].setParam(m_param.Ktn, m_param.Jmn, m_param.g_dis, m_param.Ts);

	for(int ch = 0; ch < 4; ch++)
		m_canMotorIF->motor[ch].set_Iq_ref(0.0f);

}


void OmniWheel::update(Cmd_t *cmd)
{

	switch(m_type)
	{
	case TYPE_INDEP_P:
		for(int i = 0; i < 4; i++)
		{
			m_cmd.omega_w[i] = cmd->omega_w[i];
			float Iq_ref = m_param.Kp * (m_cmd.omega_w[i] - m_canMotorIF->motor[i].get_omega());
			if(Iq_ref < -15.0) Iq_ref = -15.0;
			if(Iq_ref > 15.0) Iq_ref = 15.0;
			m_canMotorIF->motor[i].set_Iq_ref(Iq_ref);
		}
		break;

	case TYPE_INDEP_P_DOB:
		for(int i = 0; i < 4; i++)
		{
			m_cmd.omega_w[i] = cmd->omega_w[i];
			float error = m_cmd.omega_w[i] - m_canMotorIF->motor[i].get_omega();
			float estTorque = dob[i].update(m_canMotorIF->motor[i].get_Iq_ref(), m_canMotorIF->motor[i].get_omega());
			float Iq_ref = m_param.Kp * error + estTorque / m_param.Ktn;
			if(Iq_ref < -15.0) Iq_ref = -15.0;
			if(Iq_ref > 15.0) Iq_ref = 15.0;
			m_canMotorIF->motor[i].set_Iq_ref(Iq_ref);
		}
		estTorque = dob[0].getEstTorque();
		break;

	case TYPE_P_DOB:

		for(int i = 0; i < 4; i++)
		{
			m_cmd.omega_w[i] =
					m_convMat_robot2motor[i][0] * cmd->vel_x +
					m_convMat_robot2motor[i][1] * cmd->vel_y +
					m_convMat_robot2motor[i][2] * cmd->omega;
			float error = m_cmd.omega_w[i] - m_canMotorIF->motor[i].get_omega();
			float estTorque = dob[i].update(m_canMotorIF->motor[i].get_Iq_ref(), m_canMotorIF->motor[i].get_omega());
			float Iq_ref = m_param.Kp * error + estTorque / m_param.Ktn;
			if(Iq_ref < -15.0) Iq_ref = -15.0;
			if(Iq_ref > 15.0) Iq_ref = 15.0;
			m_canMotorIF->motor[i].set_Iq_ref(Iq_ref);
		}
		break;

	default:
		break;
	}

	m_canMotorIF->send_Iq_ref();

}




