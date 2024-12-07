#pragma once

#include "PM01.hpp"

namespace SGPowerControl
{
struct PowerUpData_t
{
    /* data */
    float Cmd_Power[4];
    float Cmd_Torque[4];

    float Cur_Power[4];
    float Cur_Torque[4];

    float Cur_ALL_Power;
    float Cmd_ALL_Power;

    float Energy;
};

// class PowerLimitator
// {
//     public
// };

class PowerTask_t
{
private:

public:
    PowerUpData_t _6020_PowerData;
    PowerUpData_t _3508_PowerData;

    float ALL_Power;
    float Energy;

    void UpSteerData();
    void UpWheelData();
};

} // namespace PowerCon

#ifdef __cplusplus
extern "C"
{
#endif

    void PowerTask(void *argument);
    void RLSTask(void *argument);

#ifdef __cplusplus
}
#endif
