#ifndef _NEWHIL_H_
#define _NEWHIL_H_
//using namespace std;
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
        uint8_t _ff = 0;
        if (_Serial->readByte(&_ff) != 0)
            return 0; // error
        if (_ff != 'S')
            return 0; // error

        for (int i = 0; i < n_in; i++)
        {
            buffer_out[i] = getData();
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
        _Serial->print("A");
        for (uint8_t index = 0; index < n_out; index++)
        {
            _temp.value = value[index];
            _Serial->writeBytes(&_temp.bytes[0], sizeof(T));
        }
        _Serial->println("");
    }
};

#endif