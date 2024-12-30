#pragma once

#include <list>

namespace Event
{
    class IObserver;

    struct Dir_Data_t
    {
        /* data */
        bool Dr16;
        bool MeterPower;
        bool String[4];
        bool Wheel[4];
    };

    // 观察对象类
    class ISubject
    {
    protected:
        std::list<IObserver *> ObserverList;

    public:
        // 添加观察者
        virtual void Attach(IObserver *observer) = 0;
        // 删除观察者
        virtual void Detach(IObserver *observer) = 0;
        // 通知观察者
        virtual void Notify() = 0;
    };

    // 观察者类
    class IObserver
    {
    public:
        ISubject *sub;

        // 初始化时就绑定观察者
        IObserver(ISubject *sub) : sub(sub)
        {
            sub->Attach(this);
        }

        // 每个观察者的更新方法
        virtual void Update() = 0;
        virtual ~IObserver() = default;
    };

    class Dir : public ISubject
    {
    public:
        // 观察数据
        Dir_Data_t DirData;

        /**
         * @brief 添加观察者
         *
         * @param observers 观察者
         */
        void Attach(IObserver *observers) override
        {
            ObserverList.push_back(observers);
        }

        /**
         * @brief 删除观察者
         *
         * @param observers 观察者
         */
        void Detach(IObserver *observers) override
        {
            ObserverList.remove(observers);
        }

        /**
         * @brief 通知观察者
         *
         */
        void Notify() override
        {
            for (auto observer : ObserverList)
            {
                observer->Update();
            }
        }

    private:
        bool Dir_String();
        bool Dir_Wheel();
        bool Dir_Remote();
        bool Dir_MeterPower();

    public:
        void UpEvent();

        uint8_t GetDir_Wheel();
        uint8_t GetDir_String();
        bool GetDir_Remote();
        bool GetDir_MeterPower();
    };

    class Buzzer : public IObserver
    {
    public:
        int32_t dir_t[4];
        Buzzer(ISubject *sub) : IObserver(sub) {}

        void Update()
        {
            Dir *dir = static_cast<Dir *>(sub);

            dir_t[0] = dir->GetDir_Remote();
            dir_t[1] = dir->GetDir_MeterPower();
            dir_t[2] = dir->GetDir_String();
            dir_t[3] = dir->GetDir_Wheel();

            // if (dir->GetDir_Remote() == 1)
            // {
            //     dir_t = 0;

            // }
            // else
            // {
            //     dir_t = 1;
            // }
        }
    };

    /**
     * @brief 获取遥控器断联状态
     *
     * @return true 断联
     * @return false 通信正常
     */
    inline bool Dir::GetDir_Remote()
    {
        return DirData.Dr16;
    }

    /**
     * @brief 获取功率计断联状态
     *
     * @return true 断联
     * @return false 通信正常
     */
    inline bool Dir::GetDir_MeterPower()
    {
        return DirData.MeterPower;
    }

    /**
     * @brief 获取舵向轮断联状态
     *
     * @return uint8_t 几号轮掉线返回数子几
     */
    inline uint8_t Dir::GetDir_String()
    {
        uint8_t Dir;
        for (int i = 0; i < 4; i++)
        {
            if (DirData.String[i] == 1)
                return i + 1;
        }

        return 0;
    }

    /**
     * @brief 获取驱动轮断联状态
     *
     * @return uint8_t 几号轮掉线返回数子几
     */
    inline uint8_t Dir::GetDir_Wheel()
    {
        uint8_t Dir;
        for (int i = 0; i < 4; i++)
        {
            if (DirData.Wheel[i] == 1)
                return i + 1;
        }

        return 0;
    }
}

extern Event::Dir Dir_Event;

// extern Event::EventManager EventParse;

#ifdef __cplusplus
extern "C"
{
#endif
    void DirUpdata();
    void EventTask(void *argument);

#ifdef __cplusplus
}
#endif
