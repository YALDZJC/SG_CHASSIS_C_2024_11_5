// 提供底层驱动，主要封装各种常用函数
#include "arm_math.h"

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
    static void *sinf() { return get()->_sinf(); }

    virtual void *_sinf() { return nullptr; }

    static void *cosf() { return get()->_cosf(); }

    virtual void *_cosf() { return nullptr; }

    static void *tanf() { return get()->_tanf(); }

    virtual void *_tanf() { return nullptr; }

    static void *asinf() { return get()->_asinf(); }

    virtual void *_asinf() { return nullptr; }

    static void *acosf() { return get()->_acosf(); }

    virtual void *_acosf() { return nullptr; }

    static void *atan2f() { return get()->_atan2f(); }
    virtual void *_atan2f() { return nullptr; }
    static void *atan2f() { return get()->_atan2f(); }
    virtual void *_atan2f() { return nullptr; }
    static void *sqrt() { return get()->_sqrt(); }

    virtual void *_sqrt() { return nullptr; }

    static void *pow() { return get()->_pow(); }

    virtual void *_pow() { return nullptr; }

    static void *fabs() { return get()->_fabs(); }

    virtual void *_fabs() { return nullptr; }

    static void *abs() { return get()->_abs(); }

    virtual void *_abs() { return nullptr; }

    static void *exp() { return get()->_exp(); }

    virtual void *_exp() { return nullptr; }
};