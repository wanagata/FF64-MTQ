
#ifndef BDOT_h
#define BDOT_h

#ifndef CONTROLLER_h
#include "utility/CONTROLLER.h"
#endif // CONTROLLER.h

class BDOT : public CONTROLLER
{
public:
 
  CONTROLLER(int timesampling) : _ts(timesampling), _isset(0)
  {
  }

  void begin()
  {
    Serial.print("CONTROLLER Freq: ");
    Serial.print(this->get_freq(), 2);
  }

  void update_att(const bool is_rad, const Quaternion q, const Vector<3> rates)
  { // from ahrs
    att.quat() = q;
    att.rate() = rates;
    if (!is_rad)
    {
      att.rate().toRadians();
    }
  }

  Vector<3> runpointing()
  {

    Vector<3> euler = att.quat().toEuler();
    euler.toDegrees();
    //Serial.print("now(deg): ");
    //Serial.print(euler[0], 2);
    //Serial.print(",");
    //Serial.print(euler[1], 2);
    //Serial.print(",");
    //Serial.print(euler[2], 2);
    //Serial.print(",");

    Attitude rt_error = this->get_realtime_err(att);
    Vector<3> euler_error = (rt_error.quat().toEuler());
    euler_error.toDegrees();
    Serial.print("error(deg): ");
    Serial.print(euler_error[0], 2);
    Serial.print(",");
    Serial.print(euler_error[1], 2);
    Serial.print(",");
    Serial.print(euler_error[2], 2);
    // Serial.print();

    // = rt_error.rate();
    // euler_error.toDegrees();
    // Serial.print(",error(deg/s): ");
    // Serial.print(euler_error[0]/_ts, 2);
    // Serial.print(",");
    // Serial.print(euler_error[1]/_ts, 2);
    // Serial.print(",");
    // Serial.print(euler_error[2]/_ts, 2);
    // Serial.println();
    euler_error.toRadians();
    return euler_error;
  }

  Attitude get_realtime_err(Attitude nows)
  {
    Quaternion q_err = target_att.quat().conjugate() * nows.quat();
    Attitude rt_error(1, q_err, target_att.rate() - nows.rate());
    return rt_error;
  }

private:
  int _ts;
  bool _isset;
  Attitude att;
  Attitude target_att;
};

#endif