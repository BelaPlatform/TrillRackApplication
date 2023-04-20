#pragma once

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include "main.h"
typedef unsigned char i2c_char_t;

class I2c
{

protected:
	int i2C_address;
	ssize_t readBytes(void* buf, size_t count);
	ssize_t writeBytes(const void* buf, size_t count);
	enum { kTimeout = 10 };
	I2C_HandleTypeDef* hi2c;
public:
	I2c(){};
	I2c(I2c&&) = delete;
	int initI2C_RW(int bus, int address, int file = -1);
	int closeI2C();

	virtual ~I2c();

};

inline int I2c::initI2C_RW(int bus, int address, int fileHnd)
{
  extern I2C_HandleTypeDef hi2c2;
  hi2c = &hi2c2;
  i2C_address = address << 1;
	return 0;
}

inline int I2c::closeI2C()
{
	return 0;
}

inline ssize_t I2c::readBytes(void *buf, size_t count)
{
  HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(hi2c, i2C_address, (uint8_t*)buf, count, kTimeout);
  if(HAL_OK == ret)
    return count;
  else
    return -1;
}

inline ssize_t I2c::writeBytes(const void *buf, size_t count)
{
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, i2C_address, (uint8_t*)buf, count, kTimeout);
  if(HAL_OK == ret)
    return count;
  else
    return -1;
}

inline I2c::~I2c(){}

inline int usleep(useconds_t us)
{
  HAL_Delay((us + 999) / 1000);
  return 0;
}
