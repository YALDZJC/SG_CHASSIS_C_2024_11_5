#include "State.hpp"
#include "Variable.hpp"
#include "Dji_Motor.hpp"
#include "dr16.hpp"
#include "PID.hpp"

int a;

// 将期望值做滤波后传入轮子
void ChassisState::Tar_Updata()
{
    tar_vx.Calc(TAR_LX);
    tar_vy.Calc(TAR_LY);

    if (CONTROL_SIG == 0)
        tar_vw.Calc(TAR_RX);
}

//将运动学解算相关，并对速度与角度进行过零处理
void ChassisState::Wheel_UpData()
{
    // 对轮子进行运动学变换
    Wheel.WheelType.UpDate(tar_vx.x1, tar_vy.x1, tar_vw.x1, 8191);

    // 储存最小角判断的速度
    tar_speed[0] = Wheel.WheelType.speed[0];
    tar_speed[1] = Wheel.WheelType.speed[1];
    tar_speed[2] = Wheel.WheelType.speed[2];
    tar_speed[3] = Wheel.WheelType.speed[3];
    // 储存最小角判断的角度
    getMinPos[0] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(L_Forward_ID, Dji_Speed), &tar_speed[0], 8191, 8191);
    getMinPos[1] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(L_Back_ID   , Dji_Speed), &tar_speed[1], 8191, 8191);
    getMinPos[2] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(R_Back_ID   , Dji_Speed), &tar_speed[2], 8191, 8191);
    getMinPos[3] = Motor6020.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(R_Forward_ID, Dji_Speed), &tar_speed[3], 8191, 8191);
//    //过零处理
//    Zero_cross[0] = Motor6020.Zero_crossing_processing(getMinPos[0], Motor6020.GetEquipData(L_Forward_ID, Dji_Speed), 8191);
//    Zero_cross[1] = Motor6020.Zero_crossing_processing(getMinPos[1], Motor6020.GetEquipData(L_Back_ID   , Dji_Speed), 8191);
//    Zero_cross[2] = Motor6020.Zero_crossing_processing(getMinPos[2], Motor6020.GetEquipData(R_Back_ID   , Dji_Speed), 8191);
//    Zero_cross[3] = Motor6020.Zero_crossing_processing(getMinPos[3], Motor6020.GetEquipData(R_Forward_ID, Dji_Speed), 8191);
}

// 滤波器更新
void ChassisState::Filtering()
{
}

void Universal_mode::upData()
{
    a = 0;
    Tar_Updata();

    Wheel_UpData();
}

void Follow_mode::upData()
{
    a = 1;
}

void Rotating_mode::upData()
{
    a = 2;
}

void Stop_mode::upData()
{
    a = 3;
}
