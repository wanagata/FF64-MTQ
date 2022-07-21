#ifndef BDOT_h
#define BDOT_h

#ifndef IMUMATH_MATRIX_HPP
#include "utility/imumaths.h"
#endif 
using namespace imu;
class BDOT 
{
public:
  double get_freq() { return 1 / _ts; }
  int get_sample_rate() const { return _ts; }
  bool get_isset() const { return _isset; }
  BDOT(int timesampling) : _ts(timesampling), _isset(0)
  {
  }

  void begin()
  {
    Serial.print("CONTROLLER Freq: ");
    Serial.print(this->get_freq(), 2);
  }

  void update(const bool is_vec, const Vector<3> gyro,const Vector<3> mag)
  { // from ahrs
    _gyro = gyro;
    _mag = mag;
    if (!is_vec)
    {
      _mag.normalize(); 
    }
  }

  Vector<3> runbdot()
  {
    // get gyro,mag
    return this->_gyro.cross(this->_mag);
  }

private:
  int _ts;
  bool _isset;
  Vector<3> _gyro;
  Vector<3> _mag;
};

#endif