#include "driver.h"

driver::driver(uint8_t pinEnable, uint8_t pinDir, uint8_t pinPWM)
{
    _pinEnable = pinEnable;
    _pinDir = pinDir;
    _pinPWM = pinPWM; 

    // Initialize pin
    pinMode(_pinEnable, OUTPUT);
    pinMode(pinDir, OUTPUT);

    enabledriver();

}

void driver::enabledriver()
{
    digitalWrite(_pinEnable, HIGH);    
}

void driver::stopdriver()
{
    digitalWrite(_pinEnable, LOW);
}

void driver::speedPWM(int pwmVal)
{
    if (pwmVal > 0)
    {
        enabledriver();
        digitalWrite(_pinDir, LOW);
    }
    else if (pwmVal < 0)
    {
        enabledriver();
        digitalWrite(_pinDir, HIGH);

        // Taking absolute
        pwmVal = -pwmVal;
    }
    else
    {
        stopdriver();
    }
  
    analogWrite(_pinPWM, pwmVal);
}
