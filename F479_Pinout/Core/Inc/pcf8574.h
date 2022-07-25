#include "stm32f4xx_hal.h"
/*----------------------------------------------------------------
pcf8574_init(0x27<<1);
TIM1->CCR3 = 32767;
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
pcf8574_write(&hi2c1,6,1,1);
*/

#define EXP_NUM 2

char str[10];
uint8_t pcf8574_addr;
uint8_t port_state;
uint8_t txdata;
int i;
int b = 0;

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

void pcf8574_init(uint8_t exp_addr)
{
    pcf8574_addr = exp_addr;
    port_state = 0b00000000;
}

void pcf8574_setaddr(int addr)
{
    pcf8574_addr = addr;
}

void pcf8574_write(I2C_HandleTypeDef *port, uint8_t port_num, int port_state, int send_cmd)
{

    uint8_t number = int_pow(2, port_num);

    uint8_t number_negated = ~number;

    if (port_num <= 7 || port_num >= 0)
    {
        if (port_state == 1)
        {
            port_state = port_state | number;
            txdata = port_state;
        }
        if (port_state == 0)
        {
            port_state = port_state & number_negated;
            txdata = port_state;
        }
    }

    if (send_cmd == 1)
    {
        uint8_t addr_send[1];
        uint8_t port_state_send[1];
        port_state_send[0] = txdata;
        addr_send[0] = pcf8574_addr;
        uint8_t addr[1];
        addr[0] = addr_send[0];
        uint8_t tx[1];
        uint8_t txdata = port_state_send[0];
        tx[0] = txdata;
        HAL_I2C_Master_Transmit(port, addr[0], tx, 1, 100);
        // HAL_I2C_Master_Transmit_DMA(&hi2c1, addr[0], tx, 1);

        // HAL_I2C_Master_Transmit(&hi2c1,pcf8574_addr[exp_num],tx,1,100);
    }
}
