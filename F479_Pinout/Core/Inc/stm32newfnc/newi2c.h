#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
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
    void writeByte(const uint8_t data)
    {
        HAL_UART_Transmit(&_uart, &data, sizeof(data), _Timeout);
    }

    uint8_t readByte(uint8_t *buffer)
    {
        return HAL_UART_Receive(&_uart, buffer, 1, _Timeout);
    }

    void writeBytes(uint8_t data[], uint8_t len)
    {
        HAL_UART_Transmit(&_uart, data, len, _Timeout);
    }

    void print(const char text[])
    {
        uint8_t len = strlen(text);
        uint8_t buf[len];
        for (int i = 0; i < len; i++)
        {
            buf[i] = text[i];
        }
        this->writeBytes(buf, sizeof(buf));
    }
    void printI(int data)
    {
        char buf[11] = {0};
        sprintf(buf, "%d", data);
        this->print(buf);
    }
    void printF(float data)
    {
        char buf[11] = {0};
        sprintf(buf, "%.4f", data);
        this->print(buf);
    }
};
