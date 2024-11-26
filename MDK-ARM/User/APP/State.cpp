#include "State.hpp"
#include "Variable.hpp"
#include "Dji_Motor.hpp"
#include "dr16.hpp"

int a;

// 将期望值做滤波后传入轮子
void ChassisState::Tar_Updata()
{
    tar_vx.Calc(TAR_LX);
    tar_vy.Calc(TAR_LY);

    if (CONTROL_SIG == 0)
        tar_vw.Calc(TAR_RX);
}

// 将运动学解算相关，并对速度与角度进行过零处理
void ChassisState::Wheel_UpData()
{
    // 对轮子进行运动学变换
    Wheel.WheelType.UpDate(tar_vx.x1, tar_vy.x1, tar_vw.x1, 8191);

    // 储存最小角判断的速度
    Chassis_Data.tar_speed[0] = Wheel.WheelType.speed[0];
    Chassis_Data.tar_speed[1] = Wheel.WheelType.speed[1];
    Chassis_Data.tar_speed[2] = Wheel.WheelType.speed[2];
    Chassis_Data.tar_speed[3] = Wheel.WheelType.speed[3];
	
    // 储存最小角判断的角度
    Chassis_Data.getMinPos[0] = Tools.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(L_Forward_ID, Dji_Speed), &Chassis_Data.tar_speed[0], 8191, 8191);
    Chassis_Data.getMinPos[1] = Tools.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(L_Back_ID,    Dji_Speed), &Chassis_Data.tar_speed[1], 8191, 8191);
    Chassis_Data.getMinPos[2] = Tools.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(R_Back_ID,    Dji_Speed), &Chassis_Data.tar_speed[2], 8191, 8191);
    Chassis_Data.getMinPos[3] = Tools.MinPosHelm(tar_vx.x1, Motor6020.GetEquipData(R_Forward_ID, Dji_Speed), &Chassis_Data.tar_speed[3], 8191, 8191);

    // 过零处理
    Chassis_Data.Zero_cross[0] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[0], Motor6020.GetEquipData(L_Forward_ID,   Dji_Angle), 8191);
    Chassis_Data.Zero_cross[1] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[1], Motor6020.GetEquipData(L_Back_ID,      Dji_Angle), 8191);
    Chassis_Data.Zero_cross[2] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[2], Motor6020.GetEquipData(R_Back_ID,      Dji_Angle), 8191);
    Chassis_Data.Zero_cross[3] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[3], Motor6020.GetEquipData(R_Forward_ID,   Dji_Angle), 8191);
}

// 滤波器更新
void ChassisState::Filtering()
{
    td_6020_1.Calc(Motor6020.GetEquipData(L_Forward_ID, Dji_Speed));
    td_6020_2.Calc(Motor6020.GetEquipData(L_Back_ID,    Dji_Speed));
    td_6020_3.Calc(Motor6020.GetEquipData(R_Back_ID,    Dji_Speed));
    td_6020_4.Calc(Motor6020.GetEquipData(R_Forward_ID, Dji_Speed));

    td_3508_1.Calc(Motor3508.GetEquipData(L_Forward_ID, Dji_Speed));
    td_3508_1.Calc(Motor3508.GetEquipData(L_Forward_ID, Dji_Speed));
    td_3508_1.Calc(Motor3508.GetEquipData(L_Forward_ID, Dji_Speed));
    td_3508_1.Calc(Motor3508.GetEquipData(L_Forward_ID, Dji_Speed));
}
void ChassisState::PID_Updata()
{
    pid_angle_0x205.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[0], Motor6020.GetEquipData(L_Forward_ID,  Dji_Angle), 8191);
    pid_angle_0x206.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[0], Motor6020.GetEquipData(L_Back_ID,     Dji_Angle), 8191);
    pid_angle_0x207.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[0], Motor6020.GetEquipData(R_Back_ID,     Dji_Angle), 8191);
    pid_angle_0x208.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[0], Motor6020.GetEquipData(R_Forward_ID,  Dji_Angle), 8191);

    pid_vel_0x205.GetPidPos(Kpid_6020_vel, pid_angle_0x205.pid.cout, td_6020_1.x1, 8191);
    pid_vel_0x206.GetPidPos(Kpid_6020_vel, pid_angle_0x206.pid.cout, td_6020_2.x1, 8191);
    pid_vel_0x207.GetPidPos(Kpid_6020_vel, pid_angle_0x207.pid.cout, td_6020_3.x1, 8191);
    pid_vel_0x208.GetPidPos(Kpid_6020_vel, pid_angle_0x208.pid.cout, td_6020_4.x1, 8191);
}

void ChassisState::CAN_Updata()
{
    Motor6020.setMSD(&msd_6020, pid_vel_0x205.pid.cout, Get_MOTOR_SET_ID_6020(0x205));
    Motor6020.setMSD(&msd_6020, pid_vel_0x205.pid.cout, Get_MOTOR_SET_ID_6020(0x206));
    Motor6020.setMSD(&msd_6020, pid_vel_0x205.pid.cout, Get_MOTOR_SET_ID_6020(0x207));
    Motor6020.setMSD(&msd_6020, pid_vel_0x205.pid.cout, Get_MOTOR_SET_ID_6020(0x208));


    Motor6020.Send_CAN(&msd_6020, SEND_MOTOR_ID_6020);
}
void Universal_mode::upData()
{
//	Base_UpData();
    // Tar_Updata();

    // Wheel_UpData();

    // CAN_Updata();
}

void Follow_mode::upData()
{
//	Base_UpData();
    // Tar_Updata();

    // Wheel_UpData();

    // CAN_Updata();
}

void Rotating_mode::upData()
{
    // Tar_Updata();
    
    // Wheel_UpData();

    // CAN_Updata();
}

void Stop_mode::upData()
{
    a = 3;
}
