#include "ChassisTask.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../APP/Referee/RM_RefereeSystem.h"
#include "HAL.hpp"
#include "State.hpp"
#include "Variable.hpp"
#include "cmsis_os2.h"

#include "../APP/Remote/KeyBroad.hpp"
#include "../APP/Remote/Mode.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../BSP/Dbus.hpp"
#include "../APP/UI/UI_Queue.hpp"
#include "../APP/UI/Static/darw_static.hpp"
TaskManager taskManager;

void ChassisTask(void *argument)
{
    osDelay(500);

    taskManager.addTask<Chassis_Task>();

    for (;;) {
        taskManager.updateAll();
        osDelay(2);
    }
}

//=== 状态处理器实现 ===//
class Chassis_Task::UniversalHandler : public StateHandler
{
    Chassis_Task &m_task;

public:
    explicit UniversalHandler(Chassis_Task &task)
        : m_task(task)
    {
    }

    void UniversalTarget()
    {
        auto cos_theta = HAL::cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr());
        auto sin_theta = HAL::sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr());

        tar_vx.Calc(TAR_LX * 660);
        tar_vy.Calc(TAR_LY * 660);

        Chassis_Data.vx = (tar_vx.x1 * cos_theta - tar_vy.x1 * sin_theta);
        Chassis_Data.vy = (tar_vx.x1 * sin_theta + tar_vy.x1 * cos_theta);
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作
        UniversalTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

float gyro_vel = 150;
class Chassis_Task::FollowHandler : public StateHandler
{
public:
    Chassis_Task &m_task;

    explicit FollowHandler(Chassis_Task &task)
        : m_task(task)
    {
    }

    void FllowTarget()
    {
        auto cos_theta = HAL::cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr());
        auto sin_theta = HAL::sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr());

        tar_vx.Calc(TAR_LX * 660);
        tar_vy.Calc(TAR_LY * 660);

        pid_vw.GetPidPos(Kpid_vw, 0, Gimbal_to_Chassis_Data.getEncoderAngleErr(), 10000);
        tar_vw.Calc(pid_vw.GetCout());

        Chassis_Data.vx = (tar_vx.x1 * cos_theta - tar_vy.x1 * sin_theta);
        Chassis_Data.vy = (tar_vx.x1 * sin_theta + tar_vy.x1 * cos_theta);
        Chassis_Data.vw = (tar_vw.x1);

        td_FF_Tar.Calc(TAR_LX * 660);
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作

        FllowTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};
float angle;
class Chassis_Task::KeyBoardHandler : public StateHandler
{
public:
    Chassis_Task &m_task;

    explicit KeyBoardHandler(Chassis_Task &task)
        : m_task(task)
    {
    }

    void FllowTarget()
    {
        float total_angle = Gimbal_to_Chassis_Data.getEncoderAngleErr();
        auto cos_theta    = HAL::cosf(-total_angle);
        auto sin_theta    = HAL::sinf(-total_angle);

        tar_vx.Calc(TAR_LX * 660);
        tar_vy.Calc(TAR_LY * 660);

        angle = Gimbal_to_Chassis_Data.getTargetOffsetAngle();

        if (Gimbal_to_Chassis_Data.getRotatingVel() > 0) {
            tar_vw.Calc(Gimbal_to_Chassis_Data.getRotatingVel() * 4);
        } else {
            pid_vw.GetPidPos(Kpid_vw, Gimbal_to_Chassis_Data.getTargetOffsetAngle(), total_angle, 10000);
            tar_vw.Calc(pid_vw.GetCout());
        }

        if (Gimbal_to_Chassis_Data.getShitf()) {
            Chassis_Data.now_power = 60.0f + ext_power_heat_data_0x0201.chassis_power_limit;
        } else {
//            Chassis_Data.now_power = ext_power_heat_data_0x0201.chassis_power_limit + Gimbal_to_Chassis_Data.getPower() - 5;

            Chassis_Data.now_power = Tools.clamp(ext_power_heat_data_0x0201.chassis_power_limit + Gimbal_to_Chassis_Data.getPower(), 120.0f, 20);
        }
        PowerControl.setMaxPower(Chassis_Data.now_power);

        if (Gimbal_to_Chassis_Data.getF5()) {
            UI::UI_send_queue.is_Delete_all = true;
            UI::Static::UI_static.Init();
        }

        Chassis_Data.vx = (tar_vx.x1 * cos_theta - tar_vy.x1 * sin_theta);
        Chassis_Data.vy = (tar_vx.x1 * sin_theta + tar_vy.x1 * cos_theta);
        Chassis_Data.vw = (tar_vw.x1);

        td_FF_Tar.Calc(TAR_LX * 660);
    }

    void handle() override
    {
        // 可访问m_task的私有成员进行底盘操作

        FllowTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

class Chassis_Task::RotatingHandler : public StateHandler
{
public:
    Chassis_Task &m_task;

    explicit RotatingHandler(Chassis_Task &task)
        : m_task(task)
    {
    }

    void RotatingTarget()
    {
        auto cos_theta = HAL::cosf(-Gimbal_to_Chassis_Data.getEncoderAngleErr());
        auto sin_theta = HAL::sinf(-Gimbal_to_Chassis_Data.getEncoderAngleErr());

        tar_vx.Calc(TAR_LX * 660);
        tar_vy.Calc(TAR_LY * 660);
        tar_vw.Calc(TAR_VW * 660);

        //        pid_vw.GetPidPos(Kpid_vw, 0, Gimbal_to_Chassis_Data.getEncoderAngleErr(), 10000);
        //        tar_vw.Calc(pid_vw.GetCout());

        Chassis_Data.vx = (tar_vx.x1 * cos_theta - tar_vy.x1 * sin_theta);
        Chassis_Data.vy = (tar_vx.x1 * sin_theta + tar_vy.x1 * cos_theta);
        Chassis_Data.vw = (tar_vw.x1);

        td_FF_Tar.Calc(TAR_LX * 660);
    }

    void handle() override
    {
        RotatingTarget();
        m_task.Wheel_UpData();
        m_task.Filtering();
        m_task.PID_Updata();
        m_task.CAN_Setting();
        m_task.CAN_Send();
    }

    void RotingTarget()
    {
        tar_vx.Calc(TAR_LX * 660);
        tar_vy.Calc(TAR_LY * 660);
        td_FF_Tar.Calc(TAR_LX * 660);

        if (CONTROL_SIG == 0)
            tar_vw.Calc(660);
    }
};

class Chassis_Task::StopHandler : public StateHandler
{
    Chassis_Task &m_task;

public:
    explicit StopHandler(Chassis_Task &task)
        : m_task(task)
    {
    }

    void handle() override
    {
        // 执行急停相关操作
        // Base_UpData();
        m_task.Tar_Updata();

        for (int i = 0; i < 4; i++) {
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

        m_task.CAN_Setting();
        m_task.CAN_Send();
    }
};

//=== 任务方法实现 ===//
Chassis_Task::Chassis_Task()
{
    // 初始化默认状态
    updateState();
}

void Chassis_Task::executeState()
{
    if (m_stateHandler) {
        m_stateHandler->handle();
    }
}

uint8_t state_num;
void Chassis_Task::updateState()
{
    using namespace BSP::Remote;

    auto switch_right = dr16.switchRight();
    auto switch_left  = dr16.switchLeft();

    if (Mode::Chassis::Universal()) {
        m_currentState = State::UniversalState;
    }
    if (Mode::Chassis::Follow()) {
        m_currentState = State::FollowState;
    }
    if (Mode::Chassis::Rotating()) {
        m_currentState = State::RotatingState;
    }
    if (Mode::Chassis::KeyBoard()) {
        m_currentState = State::KeyBoardState;
    }
    if (Mode::Chassis::Stop()) {
        m_currentState = State::StopState;
    }

    // 更新状态处理器
    switch (m_currentState) {
        case State::UniversalState:
            m_stateHandler = std::make_unique<UniversalHandler>(*this);
            break;
        case State::FollowState:
            m_stateHandler = std::make_unique<FollowHandler>(*this);
            break;
        case State::RotatingState:
            m_stateHandler = std::make_unique<RotatingHandler>(*this);
            break;
        case State::KeyBoardState:
            m_stateHandler = std::make_unique<KeyBoardHandler>(*this);
            break;
        case State::StopState:
            m_stateHandler = std::make_unique<StopHandler>(*this);
            break;
    }
}

// 将期望值做滤波后传入轮子
void Chassis_Task::Tar_Updata()
{
    tar_vx.Calc(TAR_LX * 660);
    tar_vy.Calc(TAR_LY * 660);
    tar_vw.Calc(TAR_RX * 660);

    Chassis_Data.vx = tar_vx.x1;
    Chassis_Data.vy = tar_vy.x1;
    Chassis_Data.vw = tar_vw.x1;

    td_FF_Tar.Calc(TAR_LX * 660);
}

float sin_t;
uint32_t ms;
float hz;
bool is_sin;
uint16_t pos;
float ude_tar;

// 将运动学解算相关，并对速度与角度进行过零处理
void Chassis_Task::Wheel_UpData()
{
    // 对轮子进行运动学变换
    Wheel.WheelType.UpDate(Chassis_Data.vx, Chassis_Data.vy, Chassis_Data.vw, 7200);

    // 储存最小角判断的速度
    for (int i = 0; i < 4; i++) {
        Chassis_Data.tar_speed[i] = Wheel.WheelType.speed[i];
    }

    // 储存最小角判断的角度
    for (int i = 0; i < 4; i++) {
        Chassis_Data.tar_angle[i] = Wheel.WheelType.angle[i];
    }

    // 进行最小角判断
    Chassis_Data.getMinPos[0] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[0] + Chassis_angle_Init_0x205,
                         Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[0], 16384, 8192);
    Chassis_Data.getMinPos[1] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[1] + Chassis_angle_Init_0x206,
                         Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[1], 16384, 8192);
    Chassis_Data.getMinPos[2] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[2] + Chassis_angle_Init_0x207,
                         Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[2], 16384, 8192);
    Chassis_Data.getMinPos[3] =
        Tools.MinPosHelm(Chassis_Data.tar_angle[3] + Chassis_angle_Init_0x208,
                         Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), &Chassis_Data.tar_speed[3], 16384, 8192);

    if (is_sin == true) {
        sin_t   = 4096 + HAL::sinf(2 * 3.1415926 * ms * 0.001 * hz) * 4000;
        ude_tar = 4096 + HAL::sinf(2 * 3.1415926 * ms * 0.001 * hz) * 4000;
        ms++;
    } else
        sin_t = pos;

    // 过零处理         //发现直接用for会使电机疯
    // for(int i = 0; i < 4; i++)
    // {
    //    Chassis_Data.Zero_cross[i] = Tools.Zero_crossing_processing(Chassis_Data.getMinPos[i],
    //    Motor6020.GetAngleFeedback(i), 8192);
    // }
    Chassis_Data.Zero_cross[0] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[0], Motor6020.GetEquipData(L_Forward_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[1] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[1], Motor6020.GetEquipData(L_Back_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[2] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[2], Motor6020.GetEquipData(R_Back_6020_ID, Dji_Angle), 8192);
    Chassis_Data.Zero_cross[3] = Tools.Zero_crossing_processing(
        Chassis_Data.getMinPos[3], Motor6020.GetEquipData(R_Forward_6020_ID, Dji_Angle), 8192);
}
// 滤波器更新
void Chassis_Task::Filtering()
{
    // 电机一般速度反馈噪声大
    for (int i = 0; i < 4; i++) {
        td_3508_speed[i].Calc(Motor3508.GetRPMFeedback(i));
    }
}

void Chassis_Task::PID_Updata()
{
    // 舵向电机更新
    for (int i = 0; i < 4; i++) {
        // 舵向电机前馈更新
        feed_6020[i].UpData(Chassis_Data.Zero_cross[i]);
        Chassis_Data.FF_Zero_cross[i] = Tools.Round_Error(feed_6020[i].cout, feed_6020[i].target_e, 8191);

        // 舵向电机角度环更新
        pid_angle_String[i].GetPidPos(Kpid_6020_angle, Chassis_Data.Zero_cross[i], Motor6020.GetAngleFeedback(i),
                                      16384.0f);
        // 舵向电机速度环更新
        pid_vel_String[i].GetPidPos(Kpid_6020_vel, pid_angle_String[i].pid.cout,
                                    Motor6020.GetRPMFeedback(i), 16384);
    }

    // 轮毂电机速度环更新
    for (int i = 0; i < 4; i++) {
        pid_vel_Wheel[i].GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[i], td_3508_speed[i].x1, 16384.0f);
    }
}

bool is_ude;
void Chassis_Task::CAN_Setting()
{
    for (int i = 0; i < 4; i++) {
        Chassis_Data.final_6020_Out[i] = pid_vel_String[i].GetCout();
    }

    // 如果，没有超功率就沿用pid输出，如果超功率就进入功率控制部分的判断
    for (int i = 0; i < 4; i++) {
        Chassis_Data.final_3508_Out[i] = pid_vel_Wheel[i].GetCout();
    }

    // 功率控制部分
    PowerControl.String_PowerData.UpScaleMaxPow(pid_angle_String, Motor6020);
    PowerControl.String_PowerData.UpCalcMaxTorque(Chassis_Data.final_6020_Out, Motor6020, pid_vel_String,
                                                  toque_const_6020);

    PowerControl.Wheel_PowerData.UpScaleMaxPow(pid_vel_Wheel, Motor3508);
    PowerControl.Wheel_PowerData.UpCalcMaxTorque(Chassis_Data.final_3508_Out, Motor3508, pid_vel_Wheel,
                                                 toque_const_3508);

    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[0], Get_MOTOR_SET_ID_6020(0x205));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[1], Get_MOTOR_SET_ID_6020(0x206));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[2], Get_MOTOR_SET_ID_6020(0x207));
    Motor6020.setMSD(&msd_6020, Chassis_Data.final_6020_Out[3], Get_MOTOR_SET_ID_6020(0x208));

    //    if (is_ude == true)
    //        Motor6020.setMSD(&msd_6020, ude_vel_demo.GetCout(), Get_MOTOR_SET_ID_6020(0x206));
    //    else
    //        Motor6020.setMSD(&msd_6020, ude_vel_demo.GetCout(), Get_MOTOR_SET_ID_6020(0x206));

    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[0], Get_MOTOR_SET_ID_3508(0x201));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[1], Get_MOTOR_SET_ID_3508(0x202));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[2], Get_MOTOR_SET_ID_3508(0x203));
    Motor3508.setMSD(&msd_3508_2006, Chassis_Data.final_3508_Out[3], Get_MOTOR_SET_ID_3508(0x204));
}

void Chassis_Task::CAN_Send()
{
    // 发送数据
    if (Send_ms == 0) {
        BSP::Power::pm01.PM01SendFun();
        Motor3508.Send_CAN_MAILBOX1(&msd_3508_2006, SEND_MOTOR_ID_3508);
    } else if (Send_ms == 1) {
        Motor6020.Send_CAN_MAILBOX0(&msd_6020, SEND_MOTOR_CurrentID_6020);
    }

    Send_ms++;
    Send_ms %= 2;

    // Tools.vofaSend(BSP::Power::pm01.cin_power,
    //                PowerControl.String_PowerData.EstimatedPower,
    //                PowerControl.Wheel_PowerData.EstimatedPower,
    //                0,
    //                0,
    //                PowerControl.String_PowerData.pMaxPower[3]);
}
