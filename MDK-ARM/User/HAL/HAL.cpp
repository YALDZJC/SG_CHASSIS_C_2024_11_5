#include "HAL.hpp"

HAL *HAL::hal = nullptr;
HAL *HAL::get()
{
    return hal;
}

bool HAL::check()
{
    return hal != nullptr;
}

bool HAL::inject(HAL *_hal)
{
    if (_hal == nullptr)
    {
        return false;
    }

    _hal->init();
    hal = _hal;
    return true;
}
void HAL::destroy()
{
    if (hal == nullptr)
        return;

    delete hal;
    hal = nullptr;
}

