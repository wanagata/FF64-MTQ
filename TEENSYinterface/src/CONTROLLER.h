
#ifndef CONTROLLER_h
#define CONTROLLER_h

#ifndef Attitude_h
#include "utility/Attitude.h"
#endif // Attitude.h

class CONTROLLER
{
public:
  double get_freq() { return 1 / _ts; }
  int get_sample_rate() const { return _ts; }
  bool get_isset() const { return _isset; }

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

  bool setsweeping(Attitude angle)
  {
    Attitude set_ang(1, att.quat() * angle.quat(), Vector<3>{0, 0, 0});
    return this->setpointing(set_ang);
  }

  bool setpointing(const Attitude target)
  {
    target_att = target;
    _isset = 1;
    return _isset;
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