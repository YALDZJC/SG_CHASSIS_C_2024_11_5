#pragma once
#include "stdxxx.hpp"

class Wheel_t
{
public:
    float speed[4];
	float angle[4];

    virtual void UpData(float vx,float vy,float vw,float MaxSpeed) = 0;
};

class Mecanum : public Wheel_t
{
    float kx = 0,ky = 0;

    void UpData(float vx,float vy,float vw,float MaxSpeed);
};

class SG : public Wheel_t
{
    float kx = 0,ky = 0;

    void UpData(float vx,float vy,float vw,float MaxSpeed);
};

//外部接口
template <class T>
class Wheel
{
public:
    T Mecanum;
	T SG; 
};
