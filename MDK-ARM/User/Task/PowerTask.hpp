#pragma once

#include "RLS.hpp"
#include "arm_math.h"
#include "Variable.hpp"

#define My_PI 3.14152653529799323
// #define toque_const 1.502e-5f
#define toque_const_3508 1.572977718013743e-6f
#define toque_const_6020 1.420745050719895e-5f
//  #define toque_coefficient  1.99688994e-6f // (20/16384)*(0.3)*(187/3591)/9.55

#define pMAX 120.0f

namespace SGPowerControl
{
    struct PowerObj
    {
    public:
        float pidOutput;    // torque current command, [-maxOutput, maxOutput], no unit
        float curAv;        // Measured angular velocity, [-maxAv, maxAv], rad/s
        float setAv;        // target angular velocity, [-maxAv, maxAv], rad/s
        float pidMaxOutput; // pid max output
    };

    class PowerUpData_t
    {
    private:

    
    public:
        // PowerUpData_t() = delete;
        Math::RLS<2> rls;

        Matrixf<2, 1> samples;
        Matrixf<2, 1> params;

        float MAXPower;
        PowerUpData_t()
            : rls(1e-5f, 0.99999f) // 使用构造函数初始化列表进行初始化
        {
        }

        /* data */
        float k1, k2, k3 = 8.50f, k0;

        float Energy;

        float EstimatedPower;
        float Cur_EstimatedPower;

        float Initial_Est_power[4];

        float EffectivePower;

        float pMaxPower[4];
        double Cmd_MaxT[4];

        void UpRLS(PID *pid, Dji_Motor &motor, const float toque_const);
        // 等比缩放的最大分配功率
        void UpScaleMaxPow(PID *pid, Dji_Motor &motor);
        // 计算应分配的力矩
        void UpCalcMaxTorque(float *final_Out, Dji_Motor &motor, PID *pid, const float toque_const);
    };

    class PowerTask_t
    {
    public:
        PowerTask_t()
        {
            Wheel_PowerData.MAXPower = 120;
            Wheel_PowerData.k1 = 2.32443824e-07;
            Wheel_PowerData.k2 = 2.32332226e-07;

            String_PowerData.MAXPower = 50 * 0.8f;
            String_PowerData.k1 = -3.58174657e-05;
            String_PowerData.k2 = 2.06377621e-07;
        }

        PowerUpData_t String_PowerData;
        PowerUpData_t Wheel_PowerData;
        inline float GetEstWheelPow()
        {
            return Wheel_PowerData.EstimatedPower;
        }
        inline float GetEstStringPow()
        {
            return String_PowerData.EstimatedPower;
        }
    };
} // namespace PowerControl
static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }

extern SGPowerControl::PowerTask_t PowerControl;

#ifdef __cplusplus
extern "C"
{
#endif

    void RLSTask(void *argument);

#ifdef __cplusplus
}
#endif
