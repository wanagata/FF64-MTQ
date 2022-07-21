
#ifndef CONTROLLER_h
#define CONTROLLER_h

#ifndef Attitude_h
#include "utility/Attitude.h"
#endif //Attitude.h

class CONTROLLER
{
public:
  CONTROLLER(int timesampling);
  double get_freq() { return 1/_ts; }
  int get_sample_rate() { return _ts; }
  void begin();

  bool setsweeping(Attitude angle);
  bool setpointing(Attitude desired);
  bool get_isset() { return _isset; }

  void runpointing();
  Attitude get_realtime_err(Attitude nows);
  Vector<3> get_rpy();

private:
  int _ts;
  bool _isset;
  Attitude att;
  Attitude target_att;
};

CONTROLLER::CONTROLLER(int timesampling) : _ts(timesampling) , _isset(0)
{
  
}

void CONTROLLER::begin()
{
  Serial.print("CONTROLLER Freq: "); 
  Serial.print(this->get_freq(),2);
}


Vector<3> CONTROLLER::get_rpy()
{ // from ahrs
  att[0] = att[0]+(Vector<3>{1,0,0})*DEG2RAD;

  return att[0];
}

bool CONTROLLER::setsweeping(Attitude angle)
{
  const Attitude incre(0,this->get_rpy(),Vector<3>(0,0,0));

  return this->setpointing(incre+angle);

}

bool CONTROLLER::setpointing(Attitude target)
{
  target_att = target;
  _isset = 1;
  return _isset;
}

void CONTROLLER::runpointing()
{
  Attitude nowrpy(0,this->get_rpy(),Vector<3>{0, 0, 0});
  Attitude rt_error = this->get_realtime_err(nowrpy);
  Vector<3> cmd_vec = (rt_error[0] * 100.0);
  Serial.print("now: ");
  Serial.print(nowrpy[0][0],2);
  Serial.print(",");
  Serial.print(nowrpy[0][1],2);
  Serial.print(",");
  Serial.print(nowrpy[0][2],2);
  Serial.print(",");
  Serial.print("error : ");
  Serial.print(rt_error[0][0],2);
  Serial.print(",");
  Serial.print(rt_error[0][1],2);
  Serial.print(",");
  Serial.print(rt_error[0][2],2);
  Serial.print(",");
  Serial.print(rt_error[1][0],2);
  Serial.print(",");
  Serial.print(rt_error[1][1],2);
  Serial.print(",");
  Serial.print(rt_error[1][2],2);
  Serial.print(", cmd :");
  Serial.print(cmd_vec[0],2);
  Serial.print(",");
  Serial.print(cmd_vec[1],2);
  Serial.print(",");
  Serial.println(cmd_vec[2],2);
}

Attitude CONTROLLER::get_realtime_err(Attitude nows)
{
  return nows - target_att;
}

#endif