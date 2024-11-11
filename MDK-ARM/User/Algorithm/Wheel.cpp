#include "Wheel.hpp"

void Mecanum::UpData(float vx,float vy,float vw,float MaxSpeed)//speed最大速度
{
	kx = ky = 0;

	if(MaxSpeed != 0)
	{
		kx = fabs(vx) / (MaxSpeed * 2);
		ky = fabs(vy) / (MaxSpeed * 2);
	}

    //解算速度差
    this->speed[0] = ((1.0 - ky) * vx + (1.0 - kx) * vy - vw);
    this->speed[1] = ((1.0 - ky) * vx - (1.0 - kx) * vy + vw);
    this->speed[2] = ((1.0 - ky) * vx + (1.0 - kx) * vy + vw);
    this->speed[3] = ((1.0 - ky) * vx - (1.0 - kx) * vy - vw);
}

void SG::UpData(float vx,float vy,float vw,float MaxSpeed)//speed最大速度
{
      //特殊角度
      float angle = 45 * 3.14 / 180; 
      float tempvx[4] = { 0 },tempvy[4] = { 0 },tempvw = 0;
      for(char i = 0;i < 4;i++)
      {
        tempvx[i] = vx;
        tempvy[i] = vy;
      }
      tempvw = -vw;
			
      tempvy[0] = tempvy[0]-tempvw*cos(angle);
      tempvy[1] = tempvy[1]-tempvw*cos(angle);
      tempvy[2] = tempvy[2]+tempvw*cos(angle);
      tempvy[3] = tempvy[3]+tempvw*cos(angle);
      //线速度vx
      tempvx[0] = tempvx[0]-tempvw*sin(angle);
      tempvx[1] = tempvx[1]+tempvw*sin(angle);
      tempvx[2] = tempvx[2]+tempvw*sin(angle);
      tempvx[3] = tempvx[3]-tempvw*sin(angle);
      //*x是比例
      this->speed[0] = sqrt(tempvy[0] * tempvy[0] + tempvx[0] * tempvx[0]) / 660.0f * MaxSpeed;
      this->speed[1] = sqrt(tempvy[1] * tempvy[1] + tempvx[1] * tempvx[1]) / 660.0f * MaxSpeed;
      this->speed[2] = sqrt(tempvy[2] * tempvy[2] + tempvx[2] * tempvx[2]) / 660.0f * MaxSpeed;
      this->speed[3] = sqrt(tempvy[3] * tempvy[3] + tempvx[3] * tempvx[3]) / 660.0f * MaxSpeed;
      //解算角度
      this->angle[0] = atan2(tempvx[0] , tempvy[0]) * 180 / 3.14 * 8191 / 360;         
      this->angle[1] = atan2(tempvx[1] , tempvy[1]) * 180 / 3.14 * 8191 / 360;
      this->angle[2] = atan2(tempvx[2] , tempvy[2]) * 180 / 3.14 * 8191 / 360;          
      this->angle[3] = atan2(tempvx[3] , tempvy[3]) * 180 / 3.14 * 8191 / 360;
    }


    