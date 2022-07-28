#include "stm32f4xx_hal.h"
#ifndef _NEWI2C_H
#define _NEWI2C_H

class NEWI2C
{
private:
    I2C_HandleTypeDef _i2c;
    I2C_TypeDef *_port;
    uint32_t _baudrate;
    uint32_t _Timeout;

public:
    NEWI2C(I2C_TypeDef *i2cport, const uint32_t Timeout, uint32_t ClockSpeed = 100000)
    {
        _port = i2cport;
        _i2c.Instance = _port;
        _i2c.Init.ClockSpeed = 100000;
        _i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
        _i2c.Init.OwnAddress1 = 0;
        _i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        _i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        _i2c.Init.OwnAddress2 = 0;
        _i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        _i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
        _Timeout = Timeout;
    }
    I2C_HandleTypeDef *getHandleTypeDef() { return &_i2c; }
    void begin()
    {
        
        if (HAL_I2C_Init(&_i2c) != HAL_OK)
        {
            //Error_Handler();
        }
    }
    void writeBytes(const uint8_t addr, uint8_t data[], uint8_t len)
    {
        HAL_I2C_Master_Transmit(&_i2c, addr, data, len, _Timeout);
    }

    uint8_t readByte(const uint8_t addr,uint8_t *buffer)
    {
        return HAL_I2C_Master_Receive(&_i2c, addr, buffer, 1, _Timeout);
    }

    void readMEMs(const uint8_t addr,const uint8_t reg,uint8_t *buffer,const uint8_t len){
         HAL_I2C_Mem_Read(&_i2c,addr,reg,1,buffer,len,_Timeout);
    }

};
#endif
