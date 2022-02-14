


#ifndef _CONTROL_LIB_HPP_
#define _CONTROL_LIB_HPP_


class LPF_1order
{

public:

	LPF_1order(float cutoff, float Ts) : m_cutoff(cutoff)
	{
		m_coeff = 1.0f / (m_cutoff * Ts + 1.0f);
		m_y = 0.0f;
	}

	LPF_1order() : m_cutoff(1.0)
	{
		m_coeff = 1.0f / (m_cutoff * 1.0 + 1.0f);
		m_y = 0.0f;
	}

	void setParam(float cutoff, float Ts)
	{
		m_cutoff = cutoff;
		m_coeff = 1.0f / (m_cutoff * Ts + 1.0f);
		m_y = 0.0f;
	}

	float update(float u)
	{
		m_y = m_y * m_coeff + u * (1.0f - m_coeff);
		return m_y;
	}

	float getY(){ return m_y; }

private:

	float m_cutoff;

	float m_coeff;

	float m_y;

};


class DOB
{

public:

	DOB(float Ktn, float Jmn, float g_dis, float Ts) : lpf(g_dis, Ts), m_Ktn(Ktn), m_Jmn(Jmn), m_gdis(g_dis), m_Ts(Ts)
	{
	}

	DOB() : m_Ktn(1.0), m_Jmn(1.0), m_gdis(1.0), m_Ts(1.0)
	{
	}

	void setParam(float Ktn, float Jmn, float g_dis, float Ts)
	{
		lpf.setParam(g_dis, Ts);
		m_Ktn = Ktn;
		m_Jmn = Jmn;
		m_gdis = g_dis;
		m_Ts = Ts;
		m_y = 0.0;
	}

	float update(float Iq_ref, float omega_m)
	{
		float LPF_in = m_Ktn * Iq_ref + m_gdis * m_Jmn * omega_m;
		m_y = lpf.update(LPF_in) - m_gdis * m_Jmn * omega_m;
		return m_y;
	}


	float getEstTorque(){ return m_y; }

private:

	LPF_1order lpf;

	float m_Ktn;
	float m_Jmn;
	float m_gdis;
	float m_Ts;
	float m_y;

};




#endif /* _CONTROL_LIB_HPP_ */


