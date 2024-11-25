#pragma once

#include "stdxxx.hpp"

// 工具箱类
class Tools_t
{
private:
    uint8_t send_str2[64];

public:
    void vofaSend(float x1, float x2, float x3, float x4, float x5, float x6);

    // 过零处理
    float Zero_crossing_processing(float expectations, float feedback, float maxpos);
    // 最小角判断
    double MinPosHelm(float expectations, float feedback, float *speed, float maxspeed, float maxpos);
    
};
