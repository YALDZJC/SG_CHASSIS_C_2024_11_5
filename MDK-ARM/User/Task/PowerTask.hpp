#pragma once

#include "RLS.hpp"
#include "arm_math.h"
#include "Variable.hpp"

#define My_PI 3.14152653529799323
#define toque_const 1.502e-5f
#define toque_coefficient 1.572977718013743e-6f
//  #define toque_coefficient  1.99688994e-6f // (20/16384)*(0.3)*(187/3591)/9.55

#define pMAX 50

namespace SGPowerControl
{
    class PowerUpData_t
    {
    private:
        float Cmd_Power[4];
        float Cmd_Torque[4];

        float Cur_Power[4];
        float Cur_Torque[4];

    public:
        // PowerUpData_t() = delete;
        Math::RLS<2> rls;

        float MAXPower;
        PowerUpData_t()
            : rls(1e-5f, 0.99999f) // 使用构造函数初始化列表进行初始化
        {
            MAXPower = 50;
					k1 = 2.99999992e-05;
					k2 = 2.49999999e-07;
        }

        /* data */
        float k1, k2, k3 = 8.50f, k0;

        float Cur_ALL_Power;
        float Cmd_ALL_Power;

        float Energy;

        float EstimatedPower;
        float EffectivePower;

        float pMaxPower[4];
        double Cmd_MaxT[4];

        Matrixf<2, 1> samples;
        Matrixf<2, 1> params;

        void UpRLS(PID *pid, Dji_Motor &motor);
        void UpCalcVariables(PID *pid, Dji_Motor &motor);

        // 等比缩放的最大分配功率
        void UpScaleMaxPow(PID *pid, Dji_Motor &motor);

        //计算应分配的力矩
        void UpCalcMaxTorque(float *final_Out, Dji_Motor &motor, PID *pid);
        float initial_give_power[4]; // initial power from PID calculation
        float initial_total_power = 0;
        float scaled_give_power[4];

        float maxPowerLimited;
        float sumPowerCmd_before_clamp;
				
				    float sumErr;

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

extern SGPowerControl::PowerTask_t PowerControl;

#ifdef __cplusplus
extern "C"
{
#endif

    void PowerTask(void *argument);
    void RLSTask(void *argument);

#ifdef __cplusplus
}
#endif
