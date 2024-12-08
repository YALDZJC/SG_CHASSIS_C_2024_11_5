#pragma once

#include "PM01.hpp"
#include "RLS.hpp"
#include "arm_math.h"

#define My_PI 3.14152653529799323

namespace SGPowerControl
{
struct PowerUpData_t
{
    // PowerUpData_t() = delete;

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

};




class PowerTask_t
{
private:

public:
    PowerUpData_t _6020_PowerData;
    PowerUpData_t _3508_PowerData;

    float ALL_Power;
    float Energy;
    // Math::RLS<2> rls;

    void UpSteerData();
    void UpWheelData();

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
