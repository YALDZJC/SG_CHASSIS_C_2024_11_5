#include "RefeeTask.hpp"
#include "../APP/UI/Static/darw_static.hpp"
#include "cmsis_os2.h"

void RefeeTask(void *argument)
{
    for (;;)
    {
//        UI::Static::UI_static.PitchLine();
        osDelay(1);
    }
}
