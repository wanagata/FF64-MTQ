#include "stm32f4xx_hal.h"
#include "stdio.h"
void i2cscanner(I2C_HandleTypeDef *i2cbus, UART_HandleTypeDef *uart)
{
    uint8_t i = 0, ret;

    uint8_t Buffer[25] = {0};
    uint8_t Space[] = " - ";
    uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
    uint8_t EndMSG[] = "Done! \r\n\r\n";

    /*-[ I2C Bus Scanning ]-*/
    HAL_UART_Transmit(uart, StartMSG, sizeof(StartMSG), 10000);
    for (i = 1; i < 128; i++)
    {
        ret = HAL_I2C_IsDeviceReady(i2cbus, (uint16_t)(i << 1), 3, 5);
        if (ret != HAL_OK) /* No ACK Received At That Address */
        {
            HAL_UART_Transmit(uart, Space, sizeof(Space), 10000);
        }
        else if (ret == HAL_OK)
        {
            sprintf(Buffer, "0x%X", i);
            HAL_UART_Transmit(uart, Buffer, sizeof(Buffer), 10000);
        }
    }
    HAL_UART_Transmit(uart, EndMSG, sizeof(EndMSG), 10000);
    /*--[ Scanning Done ]--*/ 
}