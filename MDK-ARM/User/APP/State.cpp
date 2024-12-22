#include "State.hpp"
#include "Variable.hpp"

// #include "Dji_Motor.hpp"

int a;
float measure_cmd[300];
float measure_speed[300];
float measure_curent[300];

// 创建底盘任务实例
Chassis_task chassis_task;

// 将期望值做滤波后传入轮子
void ChassisState::Tar_Updata()
{
    tar_vx.Calc(TAR_LX);
    tar_vy.Calc(TAR_LY);
    td_FF_Tar.Calc(TAR_LX);

    if (CONTROL_SIG == 0)
        tar_vw.Calc(TAR_RX);
}

float sin_t;
uint32_t ms;
float hz;
bool is_sin;
uint16_t pos;
// 将运动学解算相关，并对速度与角度进行过零处理
void ChassisState::Wheel_UpData()
{
    // 对轮子进行运动学变换
    Wheel.WheelType.UpDate(tar_vx.x1, tar_vy.x1, tar_vw.x1, 7200);

    // 储存最小角判断的速度
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.tar_speed[i] = Wheel.WheelType.speed[i];
    }

    // 储存最小角判断的角度
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.tar_angle[i] = Wheel.WheelType.angle[i];
    }

    // 进行最小角判断
    Chassis_Data.getMinPos[0] = Tools.MinPosHelm(Chassis_Data.tar_angle[0] + Chassis_angle_Init_0x205, Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[0], 16384, 8192);
    Chassis_Data.getMinPos[1] = Tools.MinPosHelm(Chassis_Data.tar_angle[1] + Chassis_angle_Init_0x206, Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[1], 16384, 8192);
    Chassis_Data.getMinPos[2] = Tools.MinPosHelm(Chassis_Data.tar_angle[2] + Chassis_angle_Init_0x207, Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[2], 16384, 8192);
    Chassis_Data.getMinPos[3] = Tools.MinPosHelm(Chassis_Data.tar_angle[3] + Chassis_angle_Init_0x208, Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[3], 16384, 8192);

    if (is_sin == true)
    {
        sin_t = 4096 + HAL::sinf(2 * 3.1415926 * ms * 0.001 * hz) * 4000;
        ms++;
    }
    else
        sin_t = pos;

    // 过零处理
    // for(int i = 0; i < 4; i++)
    // {
    //    Chassis_Data.Zero_cross[i] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[i], Motor6020.GetAngleFeedback(i), 8192);
    // }
    Chassis_Data.Zero_cross[0] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[0], Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[1] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[1], Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[2] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[2], Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[3] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[3], Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), 8192);
}
// 滤波器更新
void ChassisState::Filtering()
{
    // 电机一般速度反馈噪声大
    td_3508_1.Calc(Motor3508.GetEquipData(L_Forward_3508_ID, Dji_Speed));
    td_3508_2.Calc(Motor3508.GetEquipData(L_Back_3508_ID, Dji_Speed));
    td_3508_3.Calc(Motor3508.GetEquipData(R_Back_3508_ID, Dji_Speed));
    td_3508_4.Calc(Motor3508.GetEquipData(R_Forward_3508_ID, Dji_Speed));

    for (int i = 0; i < 4; i++)
    {
        td_3508_speed[i].Calc(Motor3508.GetRPMFeedback(i));
    }
}
void ChassisState::PID_Updata()
{
    // 舵向电机更新
    for (int i = 0; i < 4; i++)
    {
        // 舵向电机前馈更新
        feed_6020[i].UpData(Chassis_Data.Zero_cross[i]);
        Chassis_Data.FF_Zero_cross[i] = Tools.Round_Error(feed_6020[i].cout, feed_6020[i].target_e, 8191);

        // 舵向电机角度环更新
        pid_angle_String[i].GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[i], Motor6020.GetAngleFeedback(i), 16384.0f);
        // 舵向电机速度环更新
        pid_vel_String[i].GetPidPos(Kpid_6020_vel, pid_angle_String[i].pid.cout + Chassis_Data.FF_Zero_cross[i], Motor6020.GetRPMFeedback(i), 16384);
    }

    // 轮毂电机速度环更新
    for (int i = 0; i < 4; i++)
    {
        pid_vel_Wheel[i].GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[i], td_3508_speed[i].x1, 16384.0f);
    }
}

void ChassisState::CAN_Setting()
{
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.final_6020_Out[i] = pid_vel_String[i].GetCout();
    }

    // 如果，没有超功率就沿用pid输出，如果超功率就进入功率控制部分的判断
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.final_3508_Out[i] = pid_vel_Wheel[i].GetCout();
    }

    // 功率控制部分
    PowerControl.String_PowerData.UpScaleMaxPow(pid_angle_String, Motor6020);
    PowerControl.String_PowerData.UpCalcMaxTorque(Chassis_Data.final_6020_Out, Motor6020, pid_vel_String, toque_const_6020);

    PowerControl.Wheel_PowerData.UpScaleMaxPow(pid_vel_Wheel, Motor3508);
    PowerControl.Wheel_PowerData.UpCalcMaxTorque(Chassis_Data.final_3508_Out, Motor3508, pid_vel_Wheel, toque_const_3508);

    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[0], Get_MOTOR_SET_ID_6020(0x205));
    //    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[1], Get_MOTOR_SET_ID_6020(0x206));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[2], Get_MOTOR_SET_ID_6020(0x207));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[3], Get_MOTOR_SET_ID_6020(0x208));

    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[0], Get_MOTOR_SET_ID_3508(0x201));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[1], Get_MOTOR_SET_ID_3508(0x202));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[2], Get_MOTOR_SET_ID_3508(0x203));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[3], Get_MOTOR_SET_ID_3508(0x204));
}

void ChassisState::CAN_Send()
{
    // 发送数据
    if (Send_ms == 0)
    {
        Motor3508.Send_CAN_MAILBOX1(&msd_3508_2006, SEND_MOTOR_ID_3508);
    }
    else if (Send_ms == 1)
    {
        Motor6020.Send_CAN_MAILBOX0(&msd_6020, SEND_MOTOR_CurrentID_6020);
    }

    Send_ms++;
    Send_ms %= 2;

    Tools.vofaSend(MeterPower.GetPower(),
                   PowerControl.String_PowerData.EstimatedPower,
                   PowerControl.String_PowerData.pMaxPower[0],
                   PowerControl.String_PowerData.pMaxPower[1],
                   PowerControl.String_PowerData.pMaxPower[2],
                   PowerControl.String_PowerData.pMaxPower[3]);
}

void Universal_mode::upData()
{
    Base_UpData();
}

void Follow_mode::upData()
{
    Base_UpData();
    // Tar_Updata();

    // Wheel_UpData();

    // CAN_Updata();
}

void Rotating_mode::upData()
{
    Base_UpData();

    // Tar_Updata();

    // Wheel_UpData();

    // CAN_Updata();
}

void Stop_mode::upData()
{
    // Base_UpData();
    Tar_Updata();
    for (int i = 0; i < 4; i++)
    {
        Chassis_Data.final_6020_Out[i] = 0;
        Chassis_Data.final_3508_Out[i] = 0;
    }

    // PID_Updata();

    pid_vel_String[0].clearPID();
    pid_vel_String[1].clearPID();
    pid_vel_String[2].clearPID();
    pid_vel_String[3].clearPID();

    pid_vel_Wheel[0].clearPID();
    pid_vel_Wheel[1].clearPID();
    pid_vel_Wheel[2].clearPID();
    pid_vel_Wheel[3].clearPID();

    CAN_Setting();
    CAN_Send();
}

State Chassis_task::GetState()
{
    if (EventParse.DirData.Dr16)
        return Stop_State;
    if (Universal)
        return Universal_State;
    if (Follow)
        return Follow_State;
    if (Rotating)
        return Rotating_State;
    if (Stop)
        return Stop_State;

    return Stop_State;
}
