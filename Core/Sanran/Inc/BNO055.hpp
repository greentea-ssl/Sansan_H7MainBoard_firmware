
#ifndef _BNO055_HPP_
#define _BNO055_HPP_


#include "stm32h7xx_hal.h"


class BNO055
{

public:


	enum class RegAddr : uint8_t
	{
		/* Page id register definition */
		PAGE_ID = 0x07,

		/* PAGE0 REGISTER DEFINITION START*/
		CHIP_ID = 0x00,

		/* Euler data registers */
		EUL_Heading_LSB = 0x1A,

		/* Unit selection register */
		UNIT_SEL = 0x3B,

		/* Mode registers */
		OPR_MODE = 0x3D,
		PWR_MODE = 0x3E,
		SYS_TRIGGER = 0x3F,
	};

	enum class PowerMode : uint8_t
	{
		NORMAL = 0x00,
		LOWPOWER = 0x01,
		SUSPEND = 0x02,
	};

	enum class OprMode : uint8_t
	{
		CONFIGMODE = 0x00,
		ACCONLY = 0x01,
		MAGONLY = 0x02,
		GYROONLY = 0x03,
		ACCMAG = 0x04,
		ACCGYRO = 0x05,
		MAGGYRO = 0x06,
		AMG = 0x07,
		IMU = 0x08,
		COMPASS = 0x09,
		M4G = 0x0A,
		NDOF_FMC_OFF = 0x0B,
		NDOF = 0x0C,
	};

	enum class SysTrigger : uint8_t
	{
		CLK_SEL = 0b10000000,
		RST_INT = 0b01000000,
		RST_SYS = 0b00100000,
		Self_Test = 0b00000001,
	};

	enum class EulUnit : uint8_t
	{
		Degrees = 0b00000000,
		Radians = 0b00000100,
	};


	BNO055(I2C_HandleTypeDef *hi2c);

	bool getStatus(){return m_status;}

	bool init();

	bool checkChipID();

	bool reset();

	bool setOprMode(OprMode mode);

	bool setPowerMode(PowerMode mode);

	bool setPage(uint8_t page);

	bool setSysTrigger(SysTrigger data);

	bool setEulUnit(EulUnit unit){ return write(static_cast<uint8_t>(RegAddr::UNIT_SEL), static_cast<uint8_t>(unit)); }

	bool updateIMU();



	bool write(uint8_t regAddr, uint8_t data);

	bool read(uint8_t regAddr, uint8_t *data);

	bool mulRead(uint8_t regAddr, uint8_t *data, uint32_t length);


	int16_t get_IMU_roll_int16(){return m_IMU_roll_int16;}
	int16_t get_IMU_pitch_int16(){return m_IMU_pitch_int16;}
	int16_t get_IMU_yaw_int16(){return m_IMU_yaw_int16;}


	float get_IMU_roll(){return m_IMU_roll;}
	float get_IMU_pitch(){return m_IMU_pitch;}
	float get_IMU_yaw(){return m_IMU_yaw;}


private:

	I2C_HandleTypeDef *m_hi2c;

	uint8_t m_i2cAddr;

	uint32_t m_Timeout;

	uint8_t m_rxBuf[32];

	uint8_t m_txData;


	int16_t m_IMU_roll_int16;
	int16_t m_IMU_pitch_int16;
	int16_t m_IMU_yaw_int16;

	float m_IMU_roll;
	float m_IMU_pitch;
	float m_IMU_yaw;


	bool m_status;


};



#endif /* _BNO055_HPP_ */

