#pragma once
#include "../APP/State.hpp"
/**
 * @brief 底盘控制任务
 * @detail 实现移动底盘的状态控制逻辑
 */
class Chassis_Task : public Task
{
  public:
    // 专属状态枚举（使用作用域限定）
    enum class State
    {
        UniversalState, // 通用模式
        FollowState,    // 跟随模式
        RotatingState,  // 旋转模式
        KeyBoardState,  // 键盘控制模式
        StopState // 急停状态
    };

    explicit Chassis_Task();

    // 禁用拷贝和赋值
    Chassis_Task(const Chassis_Task &) = delete;
    Chassis_Task &operator=(const Chassis_Task &) = delete;

  protected:
    void executeState() override;
    void updateState() override;

  private:
    // 状态处理器实现类声明
    class UniversalHandler;
    class FollowHandler;
    class RotatingHandler;
    class KeyBoardHandler;

    class StopHandler;

    // 成员变量
    State m_currentState = State::UniversalState;
    std::unique_ptr<StateHandler> m_stateHandler; // 当前状态处理器

    void Tar_Updata();

    void Wheel_UpData();

    void Filtering();

    void PID_Updata();

    void CAN_Setting();

    void CAN_Send();

    void Base_UpData()
    {
        Tar_Updata();
        Wheel_UpData();
        Filtering();

        PID_Updata();

        CAN_Setting();
        CAN_Send();
    }
};

#ifdef __cplusplus
extern "C"
{
#endif

    void ChassisTask(void *argument);

#ifdef __cplusplus
}
#endif
