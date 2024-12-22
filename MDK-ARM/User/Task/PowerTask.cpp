#include "PowerTask.hpp"
#include "cmsis_os2.h"
#include "Variable.hpp"
#include "Tools.hpp"
#include "math.h"

using namespace SGPowerControl;
SGPowerControl::PowerTask_t PowerControl;

uint32_t pm01_ms = 0;

void PowerTask(void *argument)
{
    for (;;)
    {
        osDelay(1);
    }
}

float W2, T2;
float EffectivePower_t;
void RLSTask(void *argument)
{
    for (;;)
    {
        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, Motor3508, toque_const_3508);
        PowerControl.String_PowerData.UpRLS(pid_vel_String, Motor6020, toque_const_6020);

        osDelay(1);
    }
}

void PowerUpData_t::UpRLS(PID *pid, Dji_Motor &motor, const float toque_const)
{
    EffectivePower = 0;
    samples[0][0] = 0;
    samples[1][0] = 0;

    for (int i = 0; i < 4; i++)
    {
        EffectivePower += motor.GetEquipData_for(i, Dji_Torque) * motor.GetEquipData_for(i, Dji_Speed) * toque_const;

        samples[0][0] += motor.GetEquipData_for(i, Dji_Speed) * motor.GetEquipData_for(i, Dji_Speed);
        samples[1][0] += motor.GetEquipData_for(i, Dji_Torque) * motor.GetEquipData_for(i, Dji_Torque);
    }

    if (EventParse.DirData.MeterPower == false)
    {
        params = rls.update(samples, MeterPower.GetPower() - EffectivePower - k3);
        k1 = fmax(params[0][0], 1e-15f); // In case the k1 diverge to negative number
        k2 = fmax(params[1][0], 1e-15f); // In case the k2 diverge to negative number
    }
    Cur_EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + EffectivePower + k3;

    EstimatedPower = 0;
    for (int i = 0; i < 4; i++)
    {
        Initial_Est_power[i] = k1 * motor.GetEquipData_for(i, Dji_Speed) * motor.GetEquipData_for(i, Dji_Speed) +
                               k2 * pid[i].pid.cout * toque_const * pid[i].pid.cout * toque_const +
                               pid[i].pid.cout * motor.GetEquipData_for(i, Dji_Speed) * toque_const + k3/4.0;

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
            float omega = motor.GetEquipData_for(i, Dji_Speed);

            float A = k2;
            float B = toque_const * omega;
            float C = k1 * omega * omega + k3 / 4 - pMaxPower[i];

            delta = (B * B) - 4.0f * A * C;
            if (delta <= 0)
            {
                Cmd_MaxT[i] = -B / (2.0f * A);
            }

            Cmd_MaxT[i] = pid[i].GetCout() > 0.0f ? (-B + sqrtf(delta)) / (2.0f * A)
                                                  : (-B - sqrtf(delta)) / (2.0f * A);

            Cmd_MaxT[i] = Tools.clamp(Cmd_MaxT[i], 16384.0f);

            final_Out[i] = Cmd_MaxT[i];
        }
    }

}


// void PowerUpData_t::UpRLS(PID *pid, Dji_Motor &motor)
// {
//     EffectivePower = 0;
//     samples[0][0] = 0;
//     samples[1][0] = 0;

//     for (int i = 0; i < 4; i++)
//     {
//         EffectivePower += motor.GetTorqueFeedback(i) * motor.GetRPMFeedback(i) ;

//         samples[0][0] += fabsf(motor.GetRPMFeedback(i));
//         samples[1][0] += motor.GetTorqueFeedback(i) * motor.GetTorqueFeedback(i);
//     }

//     if (EventParse.DirData.MeterPower == false)
//     {
//         params = rls.update(samples, MeterPower.GetPower() - EffectivePower - k3);
//         k1 = fmax(params[0][0], 1e-10f); // In case the k1 diverge to negative number
//         k2 = fmax(params[1][0], 1e-10f); // In case the k2 diverge to negative number
//     }

//     EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + EffectivePower + k3;
// }

// float sumCmdPower = 0.0f;
// float delta = 0;
// float PowerUpData_t::GetControlledOutput(Dji_Motor &motor, PID *pid)
// {
//     k0 = toque_coefficient;

//     sumCmdPower = 0.0f;
//     float allocatablePower = MAXPower;
//     float sumPowerRequired = 0.0f;
//     float error[4];

//     for (int i = 0; i < 4; i++)
//     {
//         Cmd_Power[i] = pid[i].GetCout() * k0 * motor.GetRPMFeedback(i) + fabs(motor.GetRPMFeedback(i)) * k1 + pid[i].GetCout() * k0 * pid[i].GetCout() * k0 * k2 + k3 / static_cast<float>(4);

//         sumCmdPower += Cmd_Power[i];
//         sumPowerRequired += Cmd_Power[i];
//         error[i] = fabs(pid[i].GetErr());

//         if (floatEqual(Cmd_Power[i], 0.0f) || Cmd_Power[i] < 0.0f)
//         {
//             allocatablePower += -Cmd_Power[i];
//         }
//         else
//         {
//             sumErr += error[i];
//             sumPowerRequired += Cmd_Power[i];
//         }
//     }

//     if (sumCmdPower > MAXPower)
//     {
//         float errorConfidence = 1.0f;
//         delta = 0;

//         for (int i = 0; i < 4; i++)
//         {
//             float powerWeight_Error = fabs(pid[i].GetErr()) / sumErr;
//             float powerWeight_Prop = Cmd_Power[i] / sumPowerRequired;
//             float powerWeight = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;

//             float omega = motor.GetEquipData_for(i, Dji_Speed);

//             float A = k2;
//             float B = omega;
//             float C = k1 * omega * omega + k3 / 4 - pMaxPower[i];

//             delta = (B * B) - 4.0f * A * C;

//             float delta = motor.GetRPMFeedback(i) * motor.GetRPMFeedback(i) -
//                           4.0f * k2 * (k1 * fabs(motor.GetRPMFeedback(i)) + k3 / static_cast<float>(4) - 10);

//             if (floatEqual(delta, 0.0f)) // repeat roots
//             {
//                 newTorqueCurrent[i] = -motor.GetRPMFeedback(i) / (2.0f * k2) / k0;
//             }
//             else if (delta > 0.0f) // distinct roots
//             {
//                 newTorqueCurrent[i] = pid[i].GetCout() > 0.0f ? (-motor.GetRPMFeedback(i) + sqrtf(delta)) / (2.0f * k2)
//                                                               : (-motor.GetRPMFeedback(i) - sqrtf(delta)) / (2.0f * k2);
//             }
//             else
//             {
//                 newTorqueCurrent[i] = -motor.GetRPMFeedback(i) / (2.0f * k2) / k0;
//             }
//             //            newTorqueCurrent[i] = Tools.clamp(newTorqueCurrent[i], 16384.0f);
//         }
//     }
//     else
//     {
//         for (int i = 0; i < 4; i++)
//         {
//             newTorqueCurrent[i] = 0.0f;
//         }
//     }

//     return 0;
// }
