#include "PowerTask.hpp"
#include "cmsis_os2.h"
#include "Variable.hpp"
#include "Tools.hpp"

using namespace SGPowerControl;

uint32_t pm01_ms = 0;


void PowerTask(void *argument)
{

    for (;;)
    {


        // PowerControl.UpWheelData();
        osDelay(1);
    }
}

float W2, T2;
float EffectivePower_t;
void RLSTask(void *argument)
{
    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;

    for (;;)
    {
        PowerControl.String_PowerData.UpCalcVariables(Chassis_Data.final_3508_Out, Motor3508);
        PowerControl.Wheel_PowerData.UpCalcVariables(Chassis_Data.final_6020_Out, Motor6020);

        //        PowerControl._6020_PowerData.EstimatedPower = PowerControl._6020_PowerData.k1 * samples[0][0] + PowerControl._6020_PowerData.k2 * samples[1][0] + PowerControl._6020_PowerData.k3;

        PowerControl.String_PowerData.EffectivePower = 0;

        samples[0][0] = 0;
        samples[1][0] = 0;

        for (int i = 0; i < 4; i++)
        {
            PowerControl.String_PowerData.EffectivePower += (rpm2av(Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Speed)) * PowerControl.String_PowerData.Cur_Torque[i]);

            samples[0][0] += fabsf(rpm2av(Motor3508.GetEquipData(Get_InitID_3508(i), Dji_Speed)));
            samples[1][0] += PowerControl.String_PowerData.Cur_Torque[i] * PowerControl.String_PowerData.Cur_Torque[i];
        }

        params = PowerControl.String_PowerData.rls.update(samples, MeterPower.GetPower() - PowerControl.String_PowerData.EffectivePower);
        PowerControl.String_PowerData.k1 = fmax(params[0][0], 1e-5f); // In case the k1 diverge to negative number
        PowerControl.String_PowerData.k2 = fmax(params[1][0], 1e-5f); // In case the k2

        PowerControl.String_PowerData.EstimatedPower = PowerControl.String_PowerData.k1 * samples[0][0] + PowerControl.String_PowerData.k2 * samples[1][0] + PowerControl.String_PowerData.EffectivePower + PowerControl.String_PowerData.k3;

        osDelay(1);
    }
}

void PowerUpData_t::UpCalcVariables(float *final_Out, Dji_Motor &motor)
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
        Cmd_Torque[i] = motor.GetTorque(final_Out[i]);
    }

    for (int i = 0; i < 4; i++)
    {
        Cmd_Power[i] = Tools.GetMachinePower(Cmd_Torque[i], motor.GetEquipData_for(i, Dji_Speed));
    }

    Cmd_ALL_Power = Cmd_Power[0] + Cmd_Power[1] + Cmd_Power[2] + Cmd_Power[3];
    Cur_ALL_Power = Cur_Power[0] + Cur_Power[1] + Cur_Power[2] + Cur_Power[3];
}

void PowerUpData_t::UpRLS()
{

}
