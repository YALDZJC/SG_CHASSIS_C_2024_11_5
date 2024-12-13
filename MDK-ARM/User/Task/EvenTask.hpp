#pragma once

namespace Event
{
    struct Dir_Data_t
    {
        /* data */
        bool Dr16;
        bool MeterPower;
        bool Stree[4];
        bool Wheel[4];
    };


    class Dir
    {
    public:
        static bool Dir_Streel();
        static bool Dir_Wheel();
        static bool Dir_Remote();
        static bool Dir_MeterPower();

    public:
        static void UpEvent() 
        {
            Dir_Remote();
            Dir_Streel();
            Dir_Wheel();
            Dir_MeterPower();
        }

    public:
    };

  

    class EventManager
    {
    public:

        Dir_Data_t DirData;
    };
}

extern Event::EventManager EventParse;

#ifdef __cplusplus
extern "C"
{
#endif

    void EventTask(void *argument);

#ifdef __cplusplus
}
#endif
