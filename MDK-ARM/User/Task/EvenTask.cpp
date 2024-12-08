#include "EvenTask.hpp"
#include "Variable.hpp"
#include "cmsis_os2.h"

using namespace Event;
void EventTask(void *argument)
{
    Dir::inject(new My_Dir);
    for (;;)
    {
        Dir::Dir_Streel();
        osDelay(1);
    }
}

Dir *Dir::dir = nullptr;

Dir *Dir::get()
{
    return dir;
}

bool Dir::check()
{
    return dir != nullptr;
}

bool Dir::inject(Dir *_dir)
{
    if (_dir == nullptr)
    {
        return false;
    }

    _dir->init();
    dir = _dir;
    return true;
}

void Dir::destroy()
{
    if (dir == nullptr)
        return;

    delete dir;
    dir = nullptr;
}

bool Dir::_Dir_Streel()
{
    bool Dir = Motor6020.ISDir();

    for (int i = 0; i < 4; i++)
    {
        EventParse.DirData.Stree[i] = Motor6020.GetDir(Get_InitID_6020(i));
    }
    
    return Dir;
}

bool Dir::_Dir_Wheel()
{
    bool Dir = Motor3508.ISDir();

    for (int i = 0; i < 4; i++)
    {
        EventParse.DirData.Wheel[i] = Motor3508.GetDir(Get_InitID_3508(i));
    }

    return Dir;
}

bool Dir::_Dir_Remote()
{
    return dr16.ISDir();
}
