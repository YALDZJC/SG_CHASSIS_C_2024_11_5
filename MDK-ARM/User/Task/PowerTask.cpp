#include "PowerTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/SuperCap/SuperCap.hpp"
#include "../Task/CommunicationTask.hpp"
#include "Tools.hpp"
#include "Variable.hpp"

#include "cmsis_os2.h"
#include "math.h"
#include "stdxxx.hpp"

using namespace SGPowerControl;
SGPowerControl::PowerTask_t PowerControl;

uint32_t pm01_ms = 0;

float W2, T2;
float EffectivePower_t;
uint16_t time1 = 2;
void RLSTask(void *argument)
{
    for (;;)
    {
        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, Motor3508, toque_const_3508);
        PowerControl.String_PowerData.UpRLS(pid_vel_String, Motor6020, toque_const_6020);

yt
        float lim_cin_power = ext_power_heat_data_0x0201.chassis_power_limit - 1;
        BSP::SuperCap::cap.SetSendValue(lim_cin_power);
        BSP::SuperCap::cap.sendCAN(&hcan2, CAN_TX_MAILBOX0);

        osDelay(time1);
    }
}

void PowerUpData_t::UpRLS(PID *pid, Dji_Motor &motor, const float toque_const)
{
    EffectivePower = 0;
    samples[0][0] = 0;
    samples[1][0] = 0;

    for (int i = 0; i < 4; i++)
    {
        EffectivePower +=
            motor.GetEquipData_for(i, Dji_Torque) * motor.GetEquipData_for(i, Dji_Speed) * toque_const * rpm_to_rads;

        samples[0][0] += fabs(motor.GetEquipData_for(i, Dji_Speed));
        samples[1][0] +=
            motor.GetEquipData_for(i, Dji_Torque) * motor.GetEquipData_for(i, Dji_Torque) * toque_const * toque_const;
    }

    params = rls.update(samples, BSP::SuperCap::cap.getOutPower() - EffectivePower - k3);
    // }
    k1 = params[0][0]; // In case the k1 diverge to negative number
    k2 = params[1][0]; // In case the k2 diverge to negative number

    Cur_EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + EffectivePower + k3;

    EstimatedPower = 0;
    for (int i = 0; i < 4; i++)
    {
        Initial_Est_power[i] = pid->GetCout() * toque_const * motor.GetEquipData_for(i, Dji_Speed) * rpm_to_rads +
                               +fabs(motor.GetEquipData_for(i, Dji_Speed) * rpm_to_rads) * k1 +
                               pid->GetCout() * toque_const * pid->GetCout() * toque_const * k2 + k3 / 4.0f;

        if (Initial_Est_power[i] < 0) // negative power not included (transitory)
            continue;

        EstimatedPower += Initial_Est_power[i];
    }
}

void PowerUpData_t::UpScaleMaxPow(PID *pid, Dji_Motor &motor)
{
    float sumErr = 0;
    for (int i = 0; i < 4; i++)
    {
        sumErr += fabsf(pid[i].GetErr());
    }

    for (int i = 0; i < 4; i++)
    {
        pMaxPower[i] = MAXPower * (fabsf(pid[i].GetErr()) / sumErr);
        if (pMaxPower[i] < 0)
        {
            continue;
        }
    }
}

void PowerUpData_t::UpCalcMaxTorque(float *final_Out, Dji_Motor &motor, PID *pid, const float toque_const)
{
    if (EstimatedPower > MAXPower)
    {
        for (int i = 0; i < 4; i++)
        {
            float omega = motor.GetEquipData_for(i, Dji_Speed) * rpm_to_rads;

            float A = k2;
            float B = omega;
            float C = k1 * fabs(omega) + k3 / 4 - pMaxPower[i];

            float delta = (B * B) - 4.0f * A * C;

            if (delta <= 0)
            {
                Cmd_MaxT[i] = -B / (2.0f * A) / toque_const;
            }

            Cmd_MaxT[i] = pid[i].GetCout() > 0.0f ? (-B + sqrtf(delta)) / (2.0f * A) / toque_const
                                                  : (-B - sqrtf(delta)) / (2.0f * A) / toque_const;

            Cmd_MaxT[i] = Tools.clamp(Cmd_MaxT[i], 16384.0f, -16384.0f);

            final_Out[i] = Cmd_MaxT[i];
        }
    }
}


