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
    Vector<3> _rpy;
    Vector<3> _rate;

public:
    Attitude()
    {
        _rpy = {0, 0, 0};
        _rate = {0, 0, 0};
        _is_rad = 1;
    }
    Attitude(const bool is_rad, const Vector<3> euler, const Vector<3> rate)
        : _is_rad(is_rad), _rpy(euler), _rate(rate)
    {
        toRadians();
    }

    Attitude(const bool is_rad, const Quaternion q, const Vector<3> rate)
        : _is_rad(is_rad), _rate(rate)
    {
        _rpy = q.toEuler();
        _rate = rate;
        toRadians();
    }

    void toRadians()
    {
        if (_is_rad == 0)
        {
            _is_rad = 1;
            _rpy.toRadians();
            _rate.toRadians();
        }
    }
    Quaternion getQuat() const
    {
        Quaternion a, b, c;
        a.fromAxisAngle((Vector<3>{1, 0, 0}), _rpy[0]);
        b.fromAxisAngle((Vector<3>{0, 1, 0}), _rpy[1]);
        c.fromAxisAngle((Vector<3>{0, 0, 1}), _rpy[2]);
        return c * b * a;
    }
    Vector<3> operator[](int n) const
    {
        switch (n)
        {
        case 0:
            return _rpy;
        case 1:
            return _rate;
        }
    }
    Vector<3> &operator[](int n)
    {
        switch (n)
        {
        case 0:
            return _rpy;
        case 1:
            return _rate;
        }
    }
    Attitude operator+(const Attitude &b) const
    {
        Attitude ret;
        for (int i = 0; i < 3; i++)
        {
            ret._rpy[i] = _rpy[i] + b._rpy[i];
            ret._rate[i] = _rate[i] + b._rate[i];
        }
        return ret;
    }

    Attitude operator-(const Attitude &b) const
    {
        Attitude ret;
        for (int i = 0; i < 3; i++)
        {
            ret._rpy[i] = _rpy[i] - b._rpy[i];
            ret._rate[i] = _rate[i] - b._rate[i];
        }
        return ret;
    }
};

#endif
