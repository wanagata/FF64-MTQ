#ifndef _NEWMTQ_H_
#define _NEWMTQ_H_

#include "newINA219.h"
#include "newpcf8574.h"
#include "stm32f4xx_hal.h"
#include "stm32newfnc/newpwm.h"
#include "utility/FastPID.h"
#define sampling_rate 50 // millis
float Kp = 4, Ki = 0.01, Kd = 0.4, Hz = 1 / sampling_rate;
int output_bits = 8;
bool output_signed = true;
class NEWMTQ
{
private:
    newpcf8574 *_ioxpander;
    newINA219 *_current;
    NEWPWM *_timer;
    FastPID *_PID();//Kp, Ki, Kd, Hz, output_bits, output_signed
    uint8_t _dir_pin, _ena_pin, _tim_ch;
    int16_t _cur_pwm;
    uint16_t _pwm_period;
    float _desire_mA, _actual_mA;
    uint16_t _max_mA;
    double K_Amp_to_DipoleMoment;

public:
    NEWMTQ(newpcf8574 *IOxpander, newINA219 *CURsensor, NEWPWM *timer)
    {
        _ioxpander = IOxpander;
        _timer = timer;
        _current = CURsensor;
    }

    void mtq_config(uint16_t max_mA,double Ka){
        _max_mA = max_mA;
        K_Amp_to_DipoleMoment = Ka;
    }
    void set_current_sen(newINA219 *current) { _current = current; }
    void set_ioexander(newpcf8574 *ioexander) { _ioxpander = ioexander; }
    void set_timer(NEWPWM *timer) { _timer = timer; }
    void set_pwm_resolution(uint8_t pwm_period) { _pwm_period = pwm_period; }
    void set_tim_ch(uint8_t tim_ch) { _tim_ch = tim_ch; }
    void set_dir_pin(uint8_t dir_pin) { _dir_pin = dir_pin; }
    void set_ena_pin(uint8_t ena_pin) { _ena_pin = ena_pin; }

    void begin(uint8_t dir_pin, uint8_t ena_pin, uint8_t tim_ch, uint8_t pwm_period)
    {
        set_dir_pin(dir_pin);
        set_ena_pin(ena_pin);
        set_tim_ch(tim_ch);
        set_pwm_resolution(pwm_period);
        if (!_timer->get_is_begun()) // haven't begun yet
        {
            _timer->begin();
        }
        _current->begin();
    }

    void run_pwm(int16_t cur_pwm)
    {
        static int ab_cur_pwm = 0;
        static int dir_state = 0;
        _timer->run_ch_pwm(_tim_ch, 0);
        _ioxpander->setPIN(_ena_pin, 0);
        if (cur_pwm != 0)
        {
            _cur_pwm = cur_pwm;
            // set direction from pwm signed
            dir_state = (_cur_pwm > 0) ? 0 : 1;
            ab_cur_pwm = (!dir_state) ? _cur_pwm : -_cur_pwm;
            // saturating pwm
            ab_cur_pwm = (ab_cur_pwm > _pwm_period) ? _pwm_period : ab_cur_pwm;
        }
        else
        {
            return;
        }
        _ioxpander->setPIN(_dir_pin, dir_state);
        _timer->run_ch_pwm(_tim_ch, ab_cur_pwm);
        _ioxpander->setPIN(_ena_pin, 1);
    }
    float read_loadV()
    {
        float loadvoltage = _current->getBusVoltage_V() + (_current->getShuntVoltage_mV() / 1000);
        return loadvoltage;
    }
    float read_mA() { return _current->getCurrent_mA(); }
    newINA219 *get_current_sen() { return _current; }
    newpcf8574 *get_ioxpander() { return _ioxpander; }

    void update_sensor() { _actual_mA = read_mA(); }
    void set_desire_mA(float current_mA) { _desire_mA = current_mA; }
    float get_desire_mA() { return _desire_mA; }
    float get_actual_mA() { return _actual_mA; }
    float get_error_mA() { return _desire_mA - _actual_mA; }

    int16_t control_mA(float current_mA){
        set_desire_mA(current_mA);
        update_sensor();
        run_pwm(_PID()->step(get_error_mA(),0));
        return _cur_pwm;
    }
};

#endif
