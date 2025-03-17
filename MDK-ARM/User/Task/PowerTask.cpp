#include "PowerTask.hpp"
#include "cmsis_os2.h"
#include "Variable.hpp"
#include "Tools.hpp"
#include "math.h"
#include "../BSP/Power/PM01.hpp"
#include "../Task/CommunicationTask.hpp"
#include "stdxxx.hpp"

using namespace SGPowerControl;
SGPowerControl::PowerTask_t PowerControl;

uint32_t pm01_ms = 0;

float W2, T2;
float EffectivePower_t;
uint16_t time = 2;
void RLSTask(void *argument)
{
    for (;;) {
        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, Motor3508, toque_const_3508);
        PowerControl.String_PowerData.UpRLS(pid_vel_String, Motor6020, toque_const_6020);

        osDelay(time);
    }
}

void PowerUpData_t::UpRLS(PID *pid, Dji_Motor &motor, const float toque_const)
{
    EffectivePower = 0;
    samples[0][0]  = 0;
    samples[1][0]  = 0;

    for (int i = 0; i < 4; i++) {
        EffectivePower += motor.GetEquipData_for(i, Dji_Torque) * motor.GetEquipData_for(i, Dji_Speed) * toque_const;

        samples[0][0] += motor.GetEquipData_for(i, Dji_Speed) * motor.GetEquipData_for(i, Dji_Speed);
        samples[1][0] += motor.GetEquipData_for(i, Dji_Torque) * motor.GetEquipData_for(i, Dji_Torque);
    }

    // if (EventParse.DirData.MeterPower == false)
    // {
   if (Gimbal_to_Chassis_Data.getRotatingVel() < 110) {
       params = rls.update(samples, BSP::Power::pm01.cin_power - EffectivePower - k3);
   }
   k1 = params[0][0]; // In case the k1 diverge to negative number
   k2 = params[1][0]; // In case the k2 diverge to negative number
    // }

    Cur_EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + EffectivePower + k3;

    EstimatedPower = 0;
    for (int i = 0; i < 4; i++) {
        Initial_Est_power[i] = k1 * motor.GetEquipData_for(i, Dji_Speed) * motor.GetEquipData_for(i, Dji_Speed) + k2 * pid[i].pid.cout * toque_const * pid[i].pid.cout * toque_const + pid[i].pid.cout * motor.GetEquipData_for(i, Dji_Speed) * toque_const + k3 / 4.0f;

        if (Initial_Est_power[i] < 0) // negative power not included (transitory)
            continue;

        EstimatedPower += Initial_Est_power[i];
    }
}

void PowerUpData_t::UpScaleMaxPow(PID *pid, Dji_Motor &motor)
{
    float sumErr = 0;
    for (int i = 0; i < 4; i++) {
        sumErr += fabsf(pid[i].GetErr());
    }

    for (int i = 0; i < 4; i++) {
        pMaxPower[i] = MAXPower * (fabsf(pid[i].GetErr()) / sumErr);
        if (pMaxPower[i] < 0) {
            continue;
        }
    }
}

void PowerUpData_t::UpCalcMaxTorque(float *final_Out, Dji_Motor &motor, PID *pid, const float toque_const)
{
    if (EstimatedPower > MAXPower) {
        for (int i = 0; i < 4; i++) {
            float omega = motor.GetEquipData_for(i, Dji_Speed);

            float A = k2;
            float B = toque_const * omega;
            float C = k1 * omega * omega + k3 / 4 - pMaxPower[i];

            float delta = (B * B) - 4.0f * A * C;
            if (delta <= 0) {
                Cmd_MaxT[i] = -B / (2.0f * A);
            }

            Cmd_MaxT[i] = pid[i].GetCout() > 0.0f ? (-B + sqrtf(delta)) / (2.0f * A)
                                                  : (-B - sqrtf(delta)) / (2.0f * A);

            Cmd_MaxT[i] = Tools.clamp(Cmd_MaxT[i], 16384.0f, -16384.0f);

            final_Out[i] = Cmd_MaxT[i];
        }
    }
}
