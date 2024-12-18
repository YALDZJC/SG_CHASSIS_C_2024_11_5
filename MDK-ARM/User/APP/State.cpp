#include "State.hpp"
#include "Variable.hpp"

// #include "Dji_Motor.hpp"

int a;
float measure_cmd[300];
float measure_speed[300];
float measure_curent[300];

// 创建底盘任务实例
Chassis_task chassis_task;

float Vol2Current(float x)
{
    //    return 0.0112 * x -2.0547;
    // return 0.4362 * x + 28.7583;

    float p3 = 0.7431;
    float p4 = -160.397;

    return p3 * x + p4;
}

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
    Chassis_Data.tar_speed[0] = Wheel.WheelType.speed[0];
    Chassis_Data.tar_speed[1] = Wheel.WheelType.speed[1];
    Chassis_Data.tar_speed[2] = Wheel.WheelType.speed[2];
    Chassis_Data.tar_speed[3] = Wheel.WheelType.speed[3];

    // 储存最小角判断的速度
    Chassis_Data.tar_angle[0] = Wheel.WheelType.angle[0];
    Chassis_Data.tar_angle[1] = Wheel.WheelType.angle[1];
    Chassis_Data.tar_angle[2] = Wheel.WheelType.angle[2];
    Chassis_Data.tar_angle[3] = Wheel.WheelType.angle[3];

    // 储存最小角判断的角度
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
    Chassis_Data.Zero_cross[0] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[0], Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[1] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[1], Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[2] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[2], Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[3] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[3], Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), 8192);
}
// 滤波器更新
void ChassisState::Filtering()
{
    // 电机一般速度反馈噪声大
    td_6020_1.Calc(Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Speed));
    td_6020_2.Calc(Motor6020.GetEquipData(L_Back_6020_ID, Dji_Speed));
    td_6020_3.Calc(Motor6020.GetEquipData(R_Back_6020_ID, Dji_Speed));
    td_6020_4.Calc(Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Speed));

    td_3508_1.Calc(Motor3508.GetEquipData(L_Forward_3508_ID, Dji_Speed));
    td_3508_2.Calc(Motor3508.GetEquipData(L_Back_3508_ID, Dji_Speed));
    td_3508_3.Calc(Motor3508.GetEquipData(R_Back_3508_ID, Dji_Speed));
    td_3508_4.Calc(Motor3508.GetEquipData(R_Forward_3508_ID, Dji_Speed));

}

float kp, kd;
float p_out, D_out;
float Error;
float POWER;
int i;
void ChassisState::PID_Updata()
{

    feed_6020_1.UpData(Chassis_Data.Zero_cross[0]);
    feed_6020_2.UpData(Chassis_Data.Zero_cross[1]);
    feed_6020_3.UpData(Chassis_Data.Zero_cross[2]);
    feed_6020_4.UpData(Chassis_Data.Zero_cross[3]);

    Chassis_Data.FF_Zero_cross[0] = Tools.Round_Error(feed_6020_1.cout, feed_6020_1.target_e, 8191);
    Chassis_Data.FF_Zero_cross[1] = Tools.Round_Error(feed_6020_2.cout, feed_6020_2.target_e, 8191);
    Chassis_Data.FF_Zero_cross[2] = Tools.Round_Error(feed_6020_3.cout, feed_6020_3.target_e, 8191);
    Chassis_Data.FF_Zero_cross[3] = Tools.Round_Error(feed_6020_4.cout, feed_6020_4.target_e, 8191);

    pid_angle_0x205.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[0], Motor6020.GetEquipData(0x205, Dji_Angle), 16384);
    pid_angle_0x206.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[1], Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), 16384);
    pid_angle_0x207.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[2], Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), 16384);
    pid_angle_0x208.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[3], Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), 16384);

    pid_vel_0x205.GetPidPos(Kpid_6020_vel, pid_angle_0x205.pid.cout + Chassis_Data.FF_Zero_cross[0], Motor6020.GetEquipData(0x205, Dji_Speed), 16384);
    pid_vel_0x206.GetPidPos(Kpid_6020_vel, pid_angle_0x206.pid.cout + Chassis_Data.FF_Zero_cross[1], Motor6020.GetEquipData(0x206, Dji_Speed), 16384);
    pid_vel_0x207.GetPidPos(Kpid_6020_vel, pid_angle_0x207.pid.cout + Chassis_Data.FF_Zero_cross[2], Motor6020.GetEquipData(0x207, Dji_Speed), 16384);
    pid_vel_0x208.GetPidPos(Kpid_6020_vel, pid_angle_0x208.pid.cout + Chassis_Data.FF_Zero_cross[3], Motor6020.GetEquipData(0x208, Dji_Speed), 16384);

    pid_vel_0x201.GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[0], td_3508_1.x1, 16384.0f);
    pid_vel_0x202.GetPidPos(Kpid_3508_vel, -Chassis_Data.tar_speed[1], td_3508_2.x1, 16384.0f);
    pid_vel_0x203.GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[2], td_3508_3.x1, 16384.0f);
    pid_vel_0x204.GetPidPos(Kpid_3508_vel, -Chassis_Data.tar_speed[3], td_3508_4.x1, 16384.0f);

    Chassis_Data.final_6020_Out[0] = pid_vel_0x205.pid.cout;
    Chassis_Data.final_6020_Out[1] = pid_vel_0x206.pid.cout;
    Chassis_Data.final_6020_Out[2] = pid_vel_0x207.pid.cout;
    Chassis_Data.final_6020_Out[3] = pid_vel_0x208.pid.cout;

    Chassis_Data.final_3508_Out[0] = pid_vel_0x201.pid.cout;
    Chassis_Data.final_3508_Out[1] = pid_vel_0x202.pid.cout;
    Chassis_Data.final_3508_Out[2] = pid_vel_0x203.pid.cout;
    Chassis_Data.final_3508_Out[3] = pid_vel_0x204.pid.cout;
}

void ChassisState::CAN_Setting()
{
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[0], Get_MOTOR_SET_ID_6020(0x205));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[1], Get_MOTOR_SET_ID_6020(0x206));
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

    Tools.vofaSend(Chassis_Data.Zero_cross[0],
                   Motor6020.GetEquipData(0x205, Dji_Angle),
                   PowerControl._6020_PowerData.Cur_ALL_Power,
                   Motor6020.GetEquipData(0x206, Dji_Angle),
                   PowerControl._3508_PowerData.Cur_ALL_Power,
                   PowerControl.ALL_Power);
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

    pid_vel_0x201.clearPID();
    pid_vel_0x202.clearPID();
    pid_vel_0x203.clearPID();
    pid_vel_0x204.clearPID();

    pid_vel_0x205.clearPID();
    pid_vel_0x206.clearPID();
    pid_vel_0x207.clearPID();
    pid_vel_0x208.clearPID();

    CAN_Setting();
    CAN_Send();
}

State Chassis_task::GetState()
{
		if(EventParse.DirData.Dr16)
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
