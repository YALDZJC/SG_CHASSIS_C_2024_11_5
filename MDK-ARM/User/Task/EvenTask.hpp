#pragma once

namespace Event
{
    struct Dir_Data_t
    {
        /* data */
        bool Dr16;
        bool Stree[4];
        bool Wheel[4];
    };


    class Dir
    {
    private:
        static Dir *dir;

    public:
        static Dir *get();   // get hal instance.
        static bool check(); // check if there is a hal instance.

        static bool inject(Dir *_dir); // inject HAL instance and run hal_init.

        static void destroy();         // destroy HAL instance.
        virtual ~Dir() = default;
        virtual void UpEvent() {}

    public:
    };

    class My_Dir : public Dir
    {
    private:
        bool Dir_Streel();
        bool Dir_Wheel();
        bool Dir_Remote();

    public:
        My_Dir() = default;

        inline void UpEvent() override
        {
            Dir_Remote();
            Dir_Streel();
            Dir_Wheel();
        }
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
