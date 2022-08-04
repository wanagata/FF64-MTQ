#ifndef _NEWHIL_H_
#define _NEWHIL_H_
// using namespace std;
#include "multconvert.h"
#include "stm32newfnc/newuart.h"
// T for datatype, n_in for n inputs, n_out for n outputs
template <class T, uint8_t n_in, uint8_t n_out>
class NEWHIL
{
private:
    NEWUART *_Serial;
    typedef union
    {
        uint8_t bytes[sizeof(T)];
        T value;
    } datapack;
    enum
    {
        IN = 0,
        OUT = 1
    };
    datapack _temp;

public:
    NEWHIL(NEWUART *Serial)
    {
        _Serial = Serial;
    }

    uint8_t updateSensor(T buffer_out[])
    {
        uint8_t len = n_in * sizeof(T) + 1;
        uint8_t _buf[len] = {0};
        
        if (_Serial->readBytes(_buf, len) != 0)
            return 0; // error
        if (_buf[0] != 'S')
            return 0; // error

        for (uint8_t index = 0; index < n_in; index++)
        {
            for (uint16_t i = 0; i < sizeof(T); i++)
            {
                //1 for 'A', i for byte's index, sizeof(T) for size type
                //store data in buffer union
                _temp.bytes[i] = _buf[i + sizeof(T) * index + 1];
            }
            //convert from byte array to the T data type
            buffer_out[index] = _temp.value;
        }
        return 1;
    }

    T getData()
    {
        datapack temp;
        for (uint8_t i = 0; i < sizeof(datapack); i++)
        {
            _Serial->readByte(&temp.bytes[i]);
        }
        return temp.value;
    }

    void sendData(T value[])
    {
        uint8_t len = n_out * sizeof(T) + 2;
        uint8_t _buf[len] = {0};
        _buf[0] = 'A';
        _buf[len - 1] = '\n';

        for (uint8_t index = 0; index < n_out; index++)
        {
            _temp.value = value[index];
            for (uint16_t i = 0; i < sizeof(T); i++)
            {
                _buf[i + sizeof(T) * index + 1] = _temp.bytes[i];
            }
        }
        //_Serial->println("");
        _Serial->writeBytes(_buf, len);
    }
};

#endif
