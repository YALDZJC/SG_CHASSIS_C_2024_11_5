#pragma once

#include "PM01.hpp"
#include "RLS.hpp"
#include "arm_math.h"
#include "Dji_Motor.hpp"

#define My_PI 3.14152653529799323

namespace SGPowerControl
{
class PowerUpData_t
{
public:
    // PowerUpData_t() = delete;
    Math::RLS<2> rls;

    PowerUpData_t()
        : rls(1e-5f, 0.99999f) // 使用构造函数初始化列表进行初始化
    {
    }

        /* data */
    float Cmd_Power[4];
    float Cmd_Torque[4];

    float Cur_Power[4];
    float Cur_Torque[4];

    float Cur_ALL_Power;
    float Cmd_ALL_Power;

    float Energy;

    float k1, k2, k3;

    float EstimatedPower;
    float EffectivePower;

    void UpRLS();

    void UpCalcVariables(float *final_Out, Dji_Motor &motor);
};


class PowerTask_t
{
private:

public:
    PowerUpData_t String_PowerData;
    PowerUpData_t Wheel_PowerData;

    float ALL_Power;
    float Energy;

    inline float GetEstWheelPow()
    {
        return Wheel_PowerData.EstimatedPower;
    }
    inline float GetEstStringPow()
    {
        return String_PowerData.EstimatedPower;
    }

    
};

} // namespace PowerCon
static inline float rpm2av(float rpm) { return rpm * My_PI / 30.0f; }

#ifdef __cplusplus
extern "C"
{
#endif

    void PowerTask(void *argument);
    void RLSTask(void *argument);

#ifdef __cplusplus
}
#endif
