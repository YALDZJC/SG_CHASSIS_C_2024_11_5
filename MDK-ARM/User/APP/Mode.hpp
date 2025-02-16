#include "../BSP/Dbus.hpp"

namespace Mode
{
using namespace Remote;

namespace Gimbal
{
/**
 * @brief 云台模式切换
 *      1: 普通模式     左上
 *      2: 发射模式     左中
 *      3: 键鼠模式     左中右中
 *      4: 停止模式     左下右下
 *
 * @param L 左开关
 * @param R 右开关
 * @return true     模式切换
 * @return false    未切换
 */
inline bool Normal(Dr16::Switch L, Dr16::Switch R)
{
    return (R == Dr16::Switch::UP);
}

inline bool Launch(Dr16::Switch L, Dr16::Switch R)
{
    return (L != Dr16::Switch::MIDDLE) && (R == Dr16::Switch::MIDDLE);
}

inline bool KeyBoard(Dr16::Switch L, Dr16::Switch R)
{
    return (L == Dr16::Switch::MIDDLE) && (R == Dr16::Switch::MIDDLE);
}

inline bool Stop(Dr16::Switch L, Dr16::Switch R)
{
    return (L == Dr16::Switch::DOWN) && (R == Dr16::Switch::DOWN);
}

} // namespace Gimbal

namespace Chassis
{
/**
 * @brief 底盘模式切换
 *      1: 万向模式     左上
 *      2: 跟随模式     左中
 *      3: 旋转模式     左下
 *      4: 键鼠模式     左中右中
 *      5: 停止模式     左下右下
 *
 * @param L 左开关
 * @param R 右开关
 * @return true     模式切换
 * @return false    未切换
 */

// 模式判断函数
inline bool Universal(Dr16::Switch L, Dr16::Switch R)
{
    return (L == Dr16::Switch::UP);
}

inline bool Follow(Dr16::Switch L, Dr16::Switch R)
{
    return (L == Dr16::Switch::MIDDLE) && (R != Dr16::Switch::MIDDLE);
}

inline bool Rotating(Dr16::Switch L, Dr16::Switch R)
{
    return (L == Dr16::Switch::DOWN);
}

inline bool KeyBoard(Dr16::Switch L, Dr16::Switch R)
{
    return (L == Dr16::Switch::MIDDLE) && (R == Dr16::Switch::MIDDLE);
}

inline bool Stop(Dr16::Switch L, Dr16::Switch R)
{
    return (L == Dr16::Switch::DOWN) && (R == Dr16::Switch::DOWN);
}
} // namespace Chassis

} // namespace Mode