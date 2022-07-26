#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
/*
 uint8_t buffer;
            uint8_t error;
            uint8_t test[] = "\nlol\r\n\r\n";
    error = Serial3.readByte(&buffer);
    if(!error){
      Serial3.printF(10.442);
    }

 */

class NEWUART
{
private:
    UART_HandleTypeDef _uart;
    USART_TypeDef *_port;
    uint32_t _baudrate;
    uint32_t _Timeout;

public:
    NEWUART(USART_TypeDef *uartport, const uint32_t Timeout, uint32_t BuadRate = 115200)
    {
        _port = uartport;
        _uart.Instance = _port;
        _uart.Init.BaudRate = BuadRate;
        _uart.Init.WordLength = UART_WORDLENGTH_8B;
        _uart.Init.StopBits = UART_STOPBITS_1;
        _uart.Init.Parity = UART_PARITY_NONE;
        _uart.Init.Mode = UART_MODE_TX_RX;
        _uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        _uart.Init.OverSampling = UART_OVERSAMPLING_16;
        _Timeout = Timeout;
        _baudrate = BuadRate;
    }
    UART_HandleTypeDef *getHandleTypeDef() { return &_uart; }
    void begin(const uint32_t BuadRate)
    {
        _uart.Init.BaudRate = BuadRate;
        if (HAL_UART_Init(&_uart) != HAL_OK)
        {
            // Error_Handler();
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
        uint8_t buf[len + 1],i;
        for (i = 0; i < len + 1; i++)
        {
            buf[i] = text[i];
        }
        buf[i] = '\0';
        this->writeBytes(buf, sizeof(buf));
    }
    void printI(int16_t data)
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
    void println(const char text[])
    {
        print(text);
        print("\r\n");
    }
};
