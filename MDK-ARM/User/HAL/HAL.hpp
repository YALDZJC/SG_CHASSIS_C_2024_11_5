// 提供底层驱动，主要封装各种常用函数

class HAL
{
private:
    static HAL *hal;

public:
    static HAL *get();   // get hal instance.
    static bool check(); // check if there is a hal instance.

    static bool inject(HAL *_hal); // inject HAL instance and run hal_init.
    static void destroy();         // destroy HAL instance.
    virtual ~HAL() = default;

    virtual void init() {}

public:
    static void delay(unsigned long _mill) { get()->_delay(_mill); }
    virtual void _delay(unsigned long _mill) {}
    static unsigned long GetTick() { return get()->_GetTick(); }
    virtual unsigned long _GetTick() { return 0; }
    // 数学函数接口封装
    static float sinf(float x) { return get()->_sinf(x); }
    virtual float _sinf(float x) { return 0; }
    static float cosf(float x) { return get()->_cosf(x); }
    virtual float _cosf(float x) { return 0; }
    static float sqrt(float in, float pOut) { return get()->_sqrt(in, pOut); }
    virtual float _sqrt(float in, float pOut) { return 0; }
};