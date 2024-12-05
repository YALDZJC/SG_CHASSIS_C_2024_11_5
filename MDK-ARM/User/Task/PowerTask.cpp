#include "PowerTask.hpp"
#include "cmsis_os2.h"
#include "Variable.hpp"

uint32_t pm01_ms = 0;
void PowerTask(void *argument)
{
//    pm01.PM01SendData(0x600, 2);

    for (;;)
    {


//        pm01_ms++;
//        switch (pm01_ms)
//        {
//        case 0: /* constant-expression */
//            pm01.PM01SendData(0x601, 0);

//            /* code */
//            break;
//        case 1:
//            pm01.PM01SendRemote(0x612);
//            break;

//        default:
//            pm01_ms = 0;
//            break;
//        }

        osDelay(1);
    }
}

void RLSTask(void *argument)
{
    for (;;)
    {

        osDelay(1);
    }
}
