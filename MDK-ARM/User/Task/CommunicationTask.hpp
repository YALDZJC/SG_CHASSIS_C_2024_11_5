#pragma once

#include "../BSP/Dbus.hpp"
#include "EvenTask.hpp"
#include "stdxxx.hpp"

// /*遥控器信号源切换*/
// /*
// 	CONTROL_SIG 0 遥控器
// 	CONTROL_SIG 1 上下板
// */
// #define CONTROL_SIG 0
// #if CONTROL_SIG == 0
// // 模式切换
// #define Universal ((L_MODE1 && R_MODE1) || (L_MODE1 && R_MODE2) || (L_MODE1 && R_MODE3)) // （1）万向模式
// #define Follow (L_MODE2 && R_MODE1) || (L_MODE2 && R_MODE3)								 // （2）底盘跟随
// #define Rotating (L_MODE3 && R_MODE1) || (L_MODE3 && R_MODE2)							 // （3）小陀螺
// #define KeyBoard (L_MODE2 && R_MODE2)													 // （4键鼠模式）
// #define Stop (L_MODE3 && R_MODE3) || EventParse.DirData.Dr16														 //
// （5停止模式）

// //期望值切换
// #define TAR_LX RC_LX
// #define TAR_LY RC_LY
// #define TAR_RX RC_RX
// #define TAR_RY RC_RY

// #elif CONTROL_SIG == 1
// //模式切换
// #define Universal (Gimbal_to_Chassis_Data.Universal_t) // （1）万向模式
// #define Follow (Gimbal_to_Chassis_Data.Follow_t)	   // （2）底盘跟随
// #define Rotating (Gimbal_to_Chassis_Data.Rotating_t)   // （3）小陀螺
// // #define KeyBoard (Gimbal_to_Chassis_Data.KeyBoard_t)											 // （4键鼠模式）
// #define Stop (Gimbal_to_Chassis_Data.stop)			   // （5停止模式）

// 	CONTROL_SIG 0 遥控器
// 	CONTROL_SIG 1 上下板

#define CONTROL_SIG 1
#if CONTROL_SIG == 0
// 期望值切换
#define TAR_LX BSP::Remote::dr16.remoteLeft().x
#define TAR_LY BSP::Remote::dr16.remoteLeft().y
#define TAR_RX BSP::Remote::dr16.remoteRight().x
#define TAR_RY BSP::Remote::dr16.remoteRight().y

#elif CONTROL_SIG == 1
// 模式切换
#define TAR_LX (Gimbal_to_Chassis_Data.getLX() - 110) / 110.0f
#define TAR_LY (Gimbal_to_Chassis_Data.getLY() - 110) / 110.0f
#define TAR_RX BSP::Remote::dr16.remoteRight().x
#define TAR_RY BSP::Remote::dr16.remoteRight().y

#endif

class Communicat_Data
{
  public:
    Communicat_Data(uint16_t size)
    {
        size_ = size;
        data_ = new uint16_t[size_];
    }
    ~Communicat_Data()
    {
        delete[] data_;
    }

  protected:
  private:
    uint16_t *data_;
    uint16_t size_;
};


namespace Communicat
{
class Gimbal_to_Chassis
{
  public:
    void Data_send();
    void Data_receive(UART_HandleTypeDef *huart);
    void Init();

  private:
    uint8_t head = 0xA5; // 帧头

	void SlidingWindowRecovery();
	
    struct __attribute__((packed)) Direction // 方向结构体
    {
        uint8_t LX;
        uint8_t LY;

        uint8_t Rotating_vel;
        float Yaw_encoder_angle_err;
			uint8_t is_v_reverse : 1;
    };

    struct __attribute__((packed)) ChassisMode // 底盘模式
    {
        uint8_t Universal_mode : 1;
        uint8_t Follow_mode : 1;
        uint8_t Rotating_mode : 1;
        uint8_t KeyBoard_mode : 1;
        uint8_t stop : 1;
    };

    struct __attribute__((packed)) UiList // 底盘模式
    {
        uint8_t MCL : 1;
        uint8_t CM : 1;
        uint8_t BP : 1;
    };


    uint8_t pData[11];

    struct Direction direction;
    struct ChassisMode chassis_mode;
    struct UiList ui_list;

    public:
    bool getUniversal()
    {
        return chassis_mode.Universal_mode;
    }

    bool getFollow()
    {
        return chassis_mode.Follow_mode;
    }

    bool getRotating()
    {
        return chassis_mode.Rotating_mode;
    }

    bool getKeyBoard()
    {
        return chassis_mode.KeyBoard_mode;
    }

    bool getStop()
    {
        return chassis_mode.stop;
    }

    uint8_t getLX()
    {
        return direction.LX;
    }

    uint8_t getLY()
    {
        return direction.LY;
    }

    float getEncoderAngleErr()
    {
        return direction.Yaw_encoder_angle_err * 0.017453;
    }

    int8_t get_is_reverse()
    {
        return direction.is_v_reverse ? 1 : -1;
    }
};

inline uint8_t getSendRc(uint16_t RcData)
{
    return (RcData / 6) - 110;
}


} // namespace Communicat
extern Communicat::Gimbal_to_Chassis Gimbal_to_Chassis_Data;

// 数据

// 将RTOS任务引至.c文件
#ifdef __cplusplus
extern "C"
{
#endif

    void CommunicationTask(void *argument);

#ifdef __cplusplus
}
#endif
