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
        virtual void init() {}
        virtual void UpData() {}

    public:
        static bool Dir_Remote() { return get()->_Dir_Remote(); }
        virtual bool _Dir_Remote();

        static bool Dir_Streel() { return get()->_Dir_Streel(); }
        virtual bool _Dir_Streel();

        static bool Dir_Wheel() { return get()->_Dir_Wheel(); }
        virtual bool _Dir_Wheel();
    };

    class EventManager
    {
    public:
        Dir_Data_t DirData;
    };

    class My_Dir : public Dir
    {
    private:


    public:
        My_Dir() = default;

        inline void UpData() override
        {
            
        }

    };
}

#ifdef __cplusplus
extern "C"
{
#endif

    void EventTask(void *argument);

#ifdef __cplusplus
}
#endif
