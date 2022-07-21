#include "Arduino.h"

class driver
{
private:
    uint8_t _pinEnable;
    uint8_t _pinDir;
    uint8_t _pinPWM;

public:
    // Setup pin
    driver(uint8_t pinEnable,uint8_t pinDir, uint8_t pinPWM);

    // Enable driver
    void enabledriver();

    // Stop driver
    void stopdriver();
    
    // Set speed    
    void speedPWM(int pwmVal);

};
