#include "State.hpp"
#include "variable.hpp"

void ChassisState::Wheel_UpData()
{
    mecanumWheel.WheelType.UpDate(td_vx.x1, td_vy.x1, td_vw.x1, 8191);
}

void ChassisState::Tar_Updata()
{
    if (CONTROL_SIG == 0)
    {
        td_vx.Calc(RC_LX);
        td_vy.Calc(RC_LY);
        td_vw.Calc(RC_RX);
    }
    else
    {
        td_vx.Calc(Gimbal_to_Chassis_Data.int16_RC_LX);
        td_vy.Calc(Gimbal_to_Chassis_Data.int16_RC_LY);
    }
}

void ChassisState::TD_Updata()
{

}

void Universal_mode::upData()
{
    if (Universal)
    {
        Wheel_UpData();
        Tar_Updata();
    }
}

void Follow_mode::upData()
{
    if (Follow)
    {

    }
}

void Rotating_mode::upData()
{
    if (Rotating)
    {

    }
}

void Stop_mode::upData()
{
    if(Stop)
    {

    }
}