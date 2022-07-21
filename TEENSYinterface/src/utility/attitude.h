#ifndef ATTITUDE_h
#define ATTITUDE_h
#ifndef RAD2DEG
#define RAD2DEG 57.2957795131
#endif
#ifndef DEG2RAD
#define DEG2RAD 0.01745329251
#endif
#include "imumaths.h"

using namespace imu;

class Attitude
{

private:
    bool _is_rad;
    Quaternion _q;
    Vector<3> _rate;

public:
    Attitude()
    {
        _q = Quaternion();
        _rate = {0, 0, 0};
        _is_rad = 1;
    }
    Attitude(const bool is_rad, const Vector<3> euler, const Vector<3> rate)
        : _is_rad(is_rad), _rate(rate)
    {
        Vector<3> _euler = euler;
        if (_is_rad == 0)
        {
            _euler.toRadians();
            _rate.toRadians();
        }
        _q = angle2quat(_euler);
    }
    Quaternion angle2quat(const Vector<3> angle) const
    {
        Quaternion a, b, c;
        a.fromAxisAngle((Vector<3>{1, 0, 0}), angle[0]);
        b.fromAxisAngle((Vector<3>{0, 1, 0}), angle[1]);
        c.fromAxisAngle((Vector<3>{0, 0, 1}), angle[2]);
        return c * b * a;
    }
    Attitude(const bool is_rad, const Quaternion q, const Vector<3> rate)
        : _is_rad(is_rad), _q(q), _rate(rate)
    {
        if (_is_rad == 0)
        {
            _rate.toRadians();
        }
    }
    Quaternion getQuat() const
    {
        return _q;
    }
    Vector<3> getRate() const
    {
        return _rate;
    }
    Quaternion &quat()
    {
        return _q;
    }
    Vector<3> &rate()
    {
        return _rate;
    }
};

#endif
