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
        PowerControl.Wheel_PowerData.UpRLS(pid_vel_Wheel, Motor3508);
        PowerControl.String_PowerData.UpRLS(pid_vel_Wheel, Motor6020);

        osDelay(1);
    }
}

void PowerUpData_t::UpCalcVariables(PID *pid, Dji_Motor &motor)
{
    for (int i = 0; i < 4; i++)
    {
        Cur_Torque[i] = motor.GetTorque(motor.GetEquipData_for(i, Dji_Torque));
    }

    for (int i = 0; i < 4; i++)
    {
        Cur_Power[i] = Tools.GetMachinePower(Cur_Torque[i], motor.GetEquipData_for(i, Dji_Speed));
    }

    for (int i = 0; i < 4; i++)
    {
        Cmd_Torque[i] = motor.GetTorque(pid[i].pid.cout);
    }

    for (int i = 0; i < 4; i++)
    {
        Cmd_Power[i] = Tools.GetMachinePower(Cmd_Torque[i], motor.GetEquipData_for(i, Dji_Speed));
    }

    // Cmd_ALL_Power = Cmd_Power[0] + Cmd_Power[1] + Cmd_Power[2] + Cmd_Power[3];
    // Cur_ALL_Power = Cur_Power[0] + Cur_Power[1] + Cur_Power[2] + Cur_Power[3];
}

void PowerUpData_t::UpRLS(PID *pid, Dji_Motor &motor)
{
    UpCalcVariables(pid, motor);

    EffectivePower = 0;
    samples[0][0] = 0;
    samples[1][0] = 0;

    for (int i = 0; i < 4; i++)
    {
        EffectivePower += motor.GetEquipData_for(i, Dji_Torque) * motor.GetEquipData_for(i, Dji_Speed) * toque_coefficient;

        samples[0][0] += motor.GetEquipData_for(i, Dji_Speed) * motor.GetEquipData_for(i, Dji_Speed);
        samples[1][0] += motor.GetEquipData_for(i, Dji_Torque) * motor.GetEquipData_for(i, Dji_Torque);
    }

    if (EventParse.DirData.MeterPower == false)
    {
        params = rls.update(samples, MeterPower.GetPower() - EffectivePower - k3);
        k1 = fmax(params[0][0], 1e-8f);  // In case the k1 diverge to negative number
        k2 = fmax(params[1][0], 1e-8f);  // In case the k2 diverge to negative number
    }
		Cur_EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] + EffectivePower + k3;

    EstimatedPower = 0;
    for (int i = 0; i < 4; i++)
    {
        Initial_Est_power[i] = k1 * motor.GetEquipData_for(i, Dji_Speed) * motor.GetEquipData_for(i, Dji_Speed) +
                               k2 * pid[i].pid.cout * toque_coefficient * pid[i].pid.cout * toque_coefficient + pid[i].pid.cout * 
															 motor.GetEquipData_for(i, Dji_Speed) * toque_coefficient + k3/4.0f;
			
			
			

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

float delta;
float omega;
float sqdeta;
float fenmui;
float A, B, C;

void PowerUpData_t::UpCalcMaxTorque(float *final_Out, Dji_Motor &motor, PID *pid)
{
    UpScaleMaxPow(pid, motor);
    if (EstimatedPower > MAXPower)
    {
        for (int i = 0; i < 4; i++)
        {
            omega = motor.GetEquipData_for(i, Dji_Speed);

            A = k1;
            B = toque_coefficient * omega;
            C = k2 * omega * omega + k3 / 4 - pMaxPower[i];

            delta = (B * B) - 4.0f * A * C;
            if (delta <= 0)
            {
                final_Out[i] = -B / (2.0f * A);
            }
            if (delta > 0)
            {
                if (pid[i].pid.cout > 0)
                {
                    float temp = (-B + std::sqrt(delta)) / (2.0f * A);
                    if (temp > 16000)
                    {
                        final_Out[i] = 16000;
                    }
                    else
                    {
                        final_Out[i] = temp;
                    }
                }
                else if (pid[i].pid.cout < 0)
                {
                    float temp = (-B - std::sqrt(delta)) / (2.0f * A);
                    if (temp < -16000)
                    {
                        final_Out[i] = -16000;
                    }
                    else
                    {
                        final_Out[i] = temp;
                    }
                }
            }
        }
    }
}