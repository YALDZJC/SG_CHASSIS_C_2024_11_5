#pragma once
#include "stdxxx.hpp"

typedef struct
{
  float speed[4];
	float angle[4];
}Wheel_Date;

class Wheel_t
{
public:
	float speed[4];
	float angle[4];

  virtual void UpDate(float vx,float vy,float vw,float MaxSpeed) = 0;
};

class Mecanum : public Wheel_t
{
private:
    float kx = 0,ky = 0;


public:

    void UpDate(float vx,float vy,float vw,float MaxSpeed);
};

class SG : public Wheel_t
{
public:    

    void UpDate(float vx,float vy,float vw,float MaxSpeed);
};

//外部接口
template <class T>
class Wheel
{
public:
    T WheelType;
};
