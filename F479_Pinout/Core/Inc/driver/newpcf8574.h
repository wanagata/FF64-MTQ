#include "stm32f4xx_hal.h"
#include "stm32newfnc/newi2c.h"

class newpcf8574
{
private:
    uint8_t _i2c_addr;
    uint8_t _pin_state;
    NEWI2C *_i2c_dev;
    char int_pow(char base, char exp)
    {
        char result = 1;
        while (exp)
        {
            if (exp & 1)
                result *= base;
            exp /= 2;
            base *= base;
        }
        return result;
    }

public:
    newpcf8574(NEWI2C *i2c_dev, int addr, uint8_t port_state = 0b00000000)
    {
    	_i2c_dev=i2c_dev;
		 _i2c_addr=addr;
		 _pin_state=port_state;
    }

    void setPIN(uint8_t port_num, uint8_t port_state)
    {
        //uint8_t number = int_pow(2, port_num);
        uint8_t num = 0b1<<port_num;
        if (port_num <= 7 || port_num >= 0)
        {
            if (port_state == 1)
            {
                port_state = port_state | num;
            }
            if (port_state == 0)
            {
                port_state = port_state & (~num);
            }

            _i2c_dev->writeBytes(_i2c_addr, &port_state, 1);
            // HAL_I2C_Master_Transmit(&hi2c1, addr[0], tx, 1, 100);
            //  HAL_I2C_Master_Transmit_DMA(&hi2c1, addr[0], tx, 1);
            //  HAL_I2C_Master_Transmit(&hi2c1,pcf8574_addr[exp_num],tx,1,100);
        }
    }
};
