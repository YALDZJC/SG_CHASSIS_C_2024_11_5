#include "../APP/UI/Dynamic/darw_dynamic.hpp"
#include "../Task/CommunicationTask.hpp"
#include "../BSP/Power/PM01.hpp"
#include "../UI_Queue.hpp"
#include "../HAL/HAL.hpp"
#include "../APP/Variable.hpp"

#include <stdio.h>
float sin_tick;
int16_t pitch_out, cap_out, speed_out;
int16_t yaw_e_rad;
int16_t yawa;
int16_t yawb;
float vel;
namespace UI::Dynamic
{
    void darw_dynamic::darw_UI()
    {
        sin_tick += 0.001;

        if (UI_send_queue.send_delet_all() == true && UI_send_queue.is_up_ui == true && UI_send_queue.send_wz() == true && UI_send_queue.send() == true) {
            yaw_e_rad =  ((Gimbal_to_Chassis_Data.getEncoderAngleErr()) / 0.017453 + 180); // 获取yaw误差

            float pitch_pos = Gimbal_to_Chassis_Data.getPitchDeg() + 80;
            pitch_out       = HAL::sinf(2 * 3.14 * sin_tick * 0.5) * 40 + 90; // 示例，pitch起始角度为90，上下40°范围
            // yaw_out         = Gimbal_to_Chassis_Data.getEncoderAngleErr();    // 示例，yaw过零处理
            cap_out = HAL::sinf(2 * 3.14 * sin_tick * 0.5) * 20 + 21;			//示例40左右，从271开始到311

            float super_cap = BSP::Power::pm01.cout_voltage * 1.60;
            //			speed_out = 
			speed_out = HAL::sinf(2 * 3.14 * sin_tick * 0.5) * 60 + 60;	//满速度为120

            vel = (fabs(Motor3508.GetRPMFeedback(0)) + fabs(Motor3508.GetRPMFeedback(1)) + fabs(Motor3508.GetRPMFeedback(1)) + fabs(Motor3508.GetRPMFeedback(3))) / 4 / 62;

            RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);

            // 绘制pitch指示
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorAmaranth);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("pitch", 1, pitch_pos, pitch_pos + 2, 960, 540, 380, 380));

            // 绘制小陀螺指示
            RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorPink);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(25);



//            yaw_e_rad = Tools.Zero_crossing_processing(yaw_e_rad + 150, yaw_e_rad, 360);
			if(yaw_e_rad >= 360)
			{
				yaw_e_rad -= 360;
			}
//            yawb = Tools.Zero_crossing_processing(yaw_e_rad + 100, yaw_e_rad, 360);

            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("gyro_Init", 2, yaw_e_rad + 210, yaw_e_rad + 160, 1600, 750, 80, 80));

            // 绘制超电能量调
            if (super_cap < 30) {
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
            } else {
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
            }

            RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("cd_Init", 3, 271, 271 + super_cap, 960, 540, 380, 380));

			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorYellow);
            RM_RefereeSystem::RM_RefereeSystemSetWidth(35);
            UI_send_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp1", 0, 1600, 690, 1600, vel + 690));
        }
    }
}