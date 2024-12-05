#pragma once

#include "PM01.hpp"

namespace SGPowerControl
{
struct PowerData_t
{
    /* data */
    float _3508Torque[4];
    float _6020Torque[4];

    float _3508Power[4];
    float _6020Power[4];

    float Wheel_Power[4];

    float ALL_Power;
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
    PowerData_t PowerData;

    void SGPowerFromCurr();
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
