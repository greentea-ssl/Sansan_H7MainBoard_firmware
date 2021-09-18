

#include "BNO055.hpp"

#include "main.h"


BNO055::BNO055(I2C_HandleTypeDef *hi2c) : m_hi2c(hi2c), m_i2cAddr(0x28), m_Timeout(10)
{
	m_status = true;

	m_status = init();

}


bool BNO055::init()
{

	if(!setPage(0)) return false;
	delay_ms(650);
	if(!checkChipID()) return false;
	delay_ms(20);
	if(!setOprMode(OprMode::CONFIGMODE)) return false;
	delay_ms(20);
	if(!reset()) return false;
	delay_ms(650);
	if(!checkChipID()) return false;
	delay_ms(20);
	if(!setPowerMode(PowerMode::NORMAL)) return false;
	delay_ms(20);
	if(!setEulUnit(EulUnit::Degrees)) return false;
	delay_ms(20);
	if(!setSysTrigger(SysTrigger::CLK_SEL)) return false;
	delay_ms(200);
	if(!setOprMode(OprMode::IMU)) return false;
	delay_ms(500);

	return true;

}



bool BNO055::checkChipID()
{

	bool ret = read(static_cast<uint8_t>(RegAddr::CHIP_ID), m_rxBuf);

	return ret && (m_rxBuf[0] == 0xA0);

}

bool BNO055::reset()
{
	return setSysTrigger(SysTrigger::RST_SYS);
}

bool BNO055::setOprMode(OprMode mode)
{
	return write(static_cast<uint8_t>(RegAddr::OPR_MODE), static_cast<uint8_t>(mode));
}

bool BNO055::setPowerMode(PowerMode mode)
{
	return write(static_cast<uint8_t>(RegAddr::PWR_MODE), static_cast<uint8_t>(mode));
}

bool BNO055::setPage(uint8_t page)
{
	return write(static_cast<uint8_t>(RegAddr::PAGE_ID), page);
}

bool BNO055::setSysTrigger(SysTrigger data)
{
	return write(static_cast<uint8_t>(RegAddr::SYS_TRIGGER), static_cast<uint8_t>(data));
}



bool BNO055::updateIMU()
{
	m_status = mulRead(static_cast<uint8_t>(RegAddr::EUL_Heading_LSB), m_rxBuf, 6);

	m_IMU_roll_int16  = ((uint16_t)m_rxBuf[3] << 8) | m_rxBuf[2];
	m_IMU_pitch_int16 = ((uint16_t)m_rxBuf[5] << 8) | m_rxBuf[4];
	m_IMU_yaw_int16   = ((uint16_t)m_rxBuf[1] << 8) | m_rxBuf[0];

	m_IMU_roll = m_IMU_roll_int16 * 2 * M_PI / 5760.0f;
	m_IMU_pitch = m_IMU_pitch_int16 * 2 * M_PI / 5760.0f;
	m_IMU_yaw = m_IMU_yaw_int16 * 2 * M_PI / 5760.0;

	return m_status;
}




bool BNO055::write(uint8_t regAddr, uint8_t data)
{
	uint8_t sendData[] = {regAddr, data};
	if(HAL_I2C_Master_Transmit(m_hi2c, m_i2cAddr << 1, sendData, 2, m_Timeout) != HAL_OK)
	{
		return false;
	}
	return true;
}

bool BNO055::read(uint8_t regAddr, uint8_t *data)
{
	if(HAL_I2C_Master_Transmit(m_hi2c, m_i2cAddr << 1, &regAddr, 1, m_Timeout) != HAL_OK)
	{
		return false;
	}

	if(HAL_I2C_Master_Receive(m_hi2c, m_i2cAddr << 1, data, 1, m_Timeout) != HAL_OK)
	{
		return false;
	}

	return true;
}

bool BNO055::mulRead(uint8_t regAddr, uint8_t *data, uint32_t length)
{
	if(HAL_I2C_Master_Transmit(m_hi2c, m_i2cAddr << 1, &regAddr, 1, m_Timeout) != HAL_OK)
	{
		return false;
	}

	if(HAL_I2C_Master_Receive(m_hi2c, m_i2cAddr << 1, data, length, m_Timeout) != HAL_OK)
	{
		return false;
	}

	return true;
}




