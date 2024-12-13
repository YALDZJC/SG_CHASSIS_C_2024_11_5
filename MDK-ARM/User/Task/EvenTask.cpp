#include "EvenTask.hpp"
#include "Variable.hpp"
#include "cmsis_os2.h"

using namespace Event;

Event::EventManager EventParse;
void EventTask(void *argument)
{
    for (;;)
    {
        Dir::UpEvent();

        osDelay(1);
    }
}
bool Dir::Dir_Remote()
{
    bool Dir = dr16.ISDir();

		EventParse.DirData.Dr16 = Dir;
	
    return Dir;
}

bool Dir::Dir_Streel()
{
    bool Dir = Motor6020.ISDir();

    for (int i = 0; i < 4; i++)
    {
        EventParse.DirData.Stree[i] = Motor6020.GetDir(Get_InitID_6020(i));
    }

    return Dir;
}

bool Dir::Dir_Wheel()
{
    bool Dir = Motor3508.ISDir();

    for (int i = 0; i < 4; i++)
    {
        EventParse.DirData.Wheel[i] = Motor3508.GetDir(Get_InitID_3508(i));
    }

    return Dir;
}

bool Dir::Dir_MeterPower()
{
    bool Dir = MeterPower.ISDir();

    EventParse.DirData.MeterPower = Dir;

    return Dir;
}


