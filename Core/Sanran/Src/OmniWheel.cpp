

#include "OmniWheel.hpp"

#include <math.h>



OmniWheel::OmniWheel(ControlType_t type, CanMotorIF *canMotorIF, Param_t *param) : m_type(type), m_canMotorIF(canMotorIF), m_param(*param)
{
}

OmniWheel::OmniWheel(ControlType_t type, CanMotorIF *canMotorIF) : m_type(type), m_canMotorIF(canMotorIF)
{

	m_param.Jmn = 1E-5;
	m_param.Kp = 0.01;
	m_param.Ktn = (60.0f / (320 * 2 * M_PI));
	m_param.Ts = 1E-3;
	m_param.g_dis = 1000;

}


void OmniWheel::update(Cmd_t *cmd)
{

	switch(m_type)
	{
	case TYPE_P_CONTROL:
		for(int i = 0; i < 4; i++)
		{
			float Iq_ref = m_param.Kp * (cmd->omega_w[i] - m_canMotorIF->motor[i].get_omega());
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




