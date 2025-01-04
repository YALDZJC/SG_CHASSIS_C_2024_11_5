#include "EvenTask.hpp"
#include "Variable.hpp"
#include "cmsis_os2.h"
#include "tim.h"
#include "../APP/Buzzer.h"
#include "../BSP/Init.hpp"
// using namespace Event;

Dir Dir_Event;

void DirUpdata()
{
    Dir_Event.UpEvent();
}

void EventTask(void *argument)
{
    osDelay(500);
    Buzzer Buzzer_Event{&Dir_Event};

    for (;;)
    {
        Dir_Event.Notify();

        osDelay(10);
    }
}
bool Dir::Dir_Remote()
{
    bool Dir = dr16.ISDir();

    DirData.Dr16 = Dir;

    return Dir;
}

bool Dir::Dir_String()
{
    bool Dir = Motor6020.ISDir();

    for (int i = 0; i < 4; i++)
    {
        DirData.String[i] = Motor6020.GetDir(Get_InitID_6020(i));
    }

    return Dir;
}

bool Dir::Dir_Wheel()
{
    bool Dir = Motor3508.ISDir();

    for (int i = 0; i < 4; i++)
    {
        DirData.Wheel[i] = Motor3508.GetDir(Get_InitID_3508(i));
    }

    return Dir;
}

bool Dir::Dir_MeterPower()
{
    bool Dir = MeterPower.ISDir();

    DirData.MeterPower = Dir;

    return Dir;
}

bool Dir::Init_Flag()
{
    DirData.InitFlag = InitFlag;
    return InitFlag;
}

/**
 * @brief 更新事件
 *
 */
void Dir::UpEvent()
{
    Dir_Remote();
    Dir_String();
    Dir_Wheel();
    Dir_MeterPower();
    Init_Flag();
}
