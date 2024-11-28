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
    Wheel.WheelType.UpDate(tar_vx.x1, tar_vy.x1, tar_vw.x1, 16384);

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
    Chassis_Data.getMinPos[0] = Tools.MinPosHelm(Chassis_Data.tar_angle[0] + Chassis_angle_Init_0x205, Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[0], 8191, 8191);
    Chassis_Data.getMinPos[1] = Tools.MinPosHelm(Chassis_Data.tar_angle[1] + Chassis_angle_Init_0x206, Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[1], 8191, 8191);
    Chassis_Data.getMinPos[2] = Tools.MinPosHelm(Chassis_Data.tar_angle[2] + Chassis_angle_Init_0x207, Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[2], 8191, 8191);
    Chassis_Data.getMinPos[3] = Tools.MinPosHelm(Chassis_Data.tar_angle[3] + Chassis_angle_Init_0x208, Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[3], 8191, 8191);

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

    td3508_torque.Calc(Motor3508.GetTorque_3508(Motor3508.GetEquipData(0x201, Dji_Torque)));
}

float kp, kd;
float p_out, D_out;

float sin_t;
uint32_t ms;
float hz;
bool is_sin;
void ChassisState::PID_Updata()
{
    if (dr16.ISDir())
    {
        Chassis_Data.tar_speed[0] = Chassis_Data.tar_speed[1] = Chassis_Data.tar_speed[2] = Chassis_Data.tar_speed[3] = 0;

        Chassis_Data.getMinPos[0] = Chassis_angle_Init_0x205;
        Chassis_Data.getMinPos[1] = Chassis_angle_Init_0x206;
        Chassis_Data.getMinPos[2] = Chassis_angle_Init_0x207;
        Chassis_Data.getMinPos[3] = Chassis_angle_Init_0x208;
    }

    // pid_angle_0x205.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[0], Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle),30000);
    // pid_angle_0x206.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[1], Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle),		30000);
    // pid_angle_0x207.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[2], Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle),		30000);
    // pid_angle_0x208.GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[3], Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle),30000);

    // pid_vel_0x205.GetPidPos(Kpid_6020_vel, pid_angle_0x205.pid.cout + feed_6020_1.UpData(Chassis_Data.Zero_cross[0]), td_6020_1.x1, 30000);
    // pid_vel_0x206.GetPidPos(Kpid_6020_vel, pid_angle_0x206.pid.cout + feed_6020_2.UpData(Chassis_Data.Zero_cross[1]), td_6020_2.x1, 30000);
    // pid_vel_0x207.GetPidPos(Kpid_6020_vel, pid_angle_0x207.pid.cout + feed_6020_3.UpData(Chassis_Data.Zero_cross[2]), td_6020_3.x1, 30000);
    // pid_vel_0x208.GetPidPos(Kpid_6020_vel, pid_angle_0x208.pid.cout + feed_6020_4.UpData(Chassis_Data.Zero_cross[3]), td_6020_4.x1, 30000);

    pid_vel_0x205.GetPidPos(Kpid_6020_vel, pid_angle_0x205.pid.cout, td_6020_1.x1, 30000);
    pid_vel_0x206.GetPidPos(Kpid_6020_vel, pid_angle_0x206.pid.cout, td_6020_2.x1, 30000);
    pid_vel_0x207.GetPidPos(Kpid_6020_vel, pid_angle_0x207.pid.cout, td_6020_3.x1, 30000);
    pid_vel_0x208.GetPidPos(Kpid_6020_vel, pid_angle_0x208.pid.cout, td_6020_4.x1, 30000);

    ms++;
    if (is_sin == true)
        sin_t = HAL::sinf(2 * 3.1415926 * ms * 0.001 * hz) * 4000;
    else
        sin_t = -Chassis_Data.tar_speed[0];

    pid_T_0x201.GetPidPos(Kpid_3508_T, Chassis_Data.tar_Torque[0], td3508_torque.x1, 16384.0f);

    pid_vel_0x201.GetPidPos(Kpid_3508_vel, sin_t, td_3508_1.x1, 16384.0f);
    pid_vel_0x202.GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[1], td_3508_2.x1, 16384.0f);
    pid_vel_0x203.GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[2], td_3508_3.x1, 16384.0f);
    pid_vel_0x204.GetPidPos(Kpid_3508_vel, -Chassis_Data.tar_speed[3], td_3508_4.x1, 16384.0f);

    Chassis_Data.final_6020_Out[0] = pid_vel_0x205.pid.cout;
    Chassis_Data.final_6020_Out[1] = pid_vel_0x206.pid.cout;
    Chassis_Data.final_6020_Out[2] = pid_T_0x207.pid.cout;
    Chassis_Data.final_6020_Out[3] = pid_vel_0x208.pid.cout;

    Chassis_Data.final_3508_Out[0] = pid_vel_0x201.pid.cout;
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
    CAN_TxHeaderTypeDef *TxHeader;

    // 发送数据
    if (Send_ms == 0)
    {
        Motor3508.Send_CAN_MAILBOX0(&msd_3508_2006, SEND_MOTOR_ID_3508);
    }
    else if (Send_ms == 1)
    {
        Motor6020.Send_CAN_MAILBOX0(&msd_6020, SEND_MOTOR_ID_6020);
    }

    Send_ms++;
    Send_ms %= 2;

    Tools.vofaSend(Motor3508.GetTorque_3508(pid_vel_0x201.pid.cout),
                   Motor3508.GetTorque_3508(Motor3508.GetEquipData(0x201, Dji_Torque)),
                   Tools.GetMachinePower(Motor3508.GetTorque_3508(Motor3508.GetEquipData(0x201, Dji_Torque)), Motor3508.GetEquipData(0x201, Dji_Speed) * 0.052074),
                   tar_vx.x1,
                   sin_t,
                   0);
}

void Universal_mode::upData()
{
    Base_UpData();
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
