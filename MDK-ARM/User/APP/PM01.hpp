#pragma once

#include "BSP_Motor.hpp"
#include "My_hal.hpp"
#include "stdxxx.hpp"

namespace BSP::Power
{
    class PM01
    {
    private:
        /*  1.	cmd：控制命令，0x00为停止指令，0x01为运行指令
            2.	power_set：输入功率设置，单位0.01w
            3.	Vcap_set：超级电容充电电压设置，单位0.01V，最大30V
            4.	Icap_set：超级电容充电电流设置，单位0.01V，最大15A
            */
        struct alignas(uint64_t) ContrlCmd {
            /* data */
            uint8_t cmd;
            uint8_t reserved;
            uint16_t power_set;
            uint16_t Vcap_set;
            uint16_t Icap_set;
        };

        struct alignas(uint64_t) uint_feedback {
            /* data */
            uint8_t fault_code;
            uint8_t state;
            uint16_t V;
            uint16_t I;
            uint16_t P;
        };

        struct alignas(uint64_t) feedback {
            /* data */
            float V;
            float I;
            float P;
        };

        feedback bat;
        feedback cap;
        feedback out;
            ContrlCmd cmd;

        int64_t data_part;

    public:
        void PM01Parse(CAN_RxHeaderTypeDef RxHeader, uint8_t RxHeaderData[])
        {
            switch (RxHeader.StdId) {
                case 0x601: {
                    /* code */
                    std::memcpy(&data_part, RxHeaderData, sizeof(uint_feedback));
                    alignas(uint64_t) auto &bat_part = *reinterpret_cast<uint_feedback *>(&data_part);

//                    bat.V = __builtin_bswap16(bat_part.V) * 0.01;
//                    bat.I = __builtin_bswap16(bat_part.I) * 0.01;
//                    bat.P = __builtin_bswap16(bat_part.P) * 0.01;
					
							bat.V =   (float)(RxHeaderData[2] << 8 | RxHeaderData[3]) * 0.01f;
							bat.I = (float)(RxHeaderData[4] << 8 | RxHeaderData[5]) * 0.01f;
							bat.P =  (float)(RxHeaderData[6] << 8 | RxHeaderData[7]) * 0.01f;

                    break;
                }
                case 0x602: {
                    /* code */
                    std::memcpy(&data_part, RxHeaderData, sizeof(uint_feedback));
                    alignas(uint64_t) auto &cap_part = *reinterpret_cast<uint_feedback *>(&data_part);
                    cap.V                            =  (float)(RxHeaderData[2] << 8 | RxHeaderData[3]) * 0.01f;
                    cap.I                            =  (float)(RxHeaderData[4] << 8 | RxHeaderData[5]) * 0.01f;
                    cap.P                            =  (float)(RxHeaderData[6] << 8 | RxHeaderData[7]) * 0.01f;
                    break;
                }
                case 0x603: {
                    /* code */
                    std::memcpy(&data_part, RxHeaderData, sizeof(uint_feedback));
                    alignas(uint64_t) auto &out_part = *reinterpret_cast<uint_feedback *>(&data_part);
                    out.V                            =  (float)(RxHeaderData[2] << 8 | RxHeaderData[3]) * 0.01f;
                    out.I                            =  (float)(RxHeaderData[4] << 8 | RxHeaderData[5]) * 0.01f;
                    out.P                            =  (float)(RxHeaderData[6] << 8 | RxHeaderData[7]) * 0.01f;
                    break;
                }
                default:
                    break;
            }
        }

        void PM01SendData(uint16_t power_set)
        {
            uint8_t SendData[8];

            uint16_t Vcap_set = 2400;
            uint16_t Icap_set = 800;

            cmd.cmd           = 0x01;
            cmd.reserved      = 0x00;
		cmd.power_set = power_set >> 8;
		cmd.power_set = power_set;

		cmd.Vcap_set  = Vcap_set >> 8;
		cmd.Vcap_set  = Vcap_set;

            cmd.Icap_set = Icap_set >> 8;
            cmd.Icap_set = Icap_set;

            std::memcpy(SendData, &cmd, sizeof(cmd));

            RM_FDorCAN_Send(&hcan2, 0x600, SendData, CAN_TX_MAILBOX2);
        }
    };

    inline PM01 pm01;
} // namespace name
