#pragma once

#include "stdxxx.hpp"

class TD {
   public:

	float u;
	float x1,x2,max_x2;
	float r,h,r2_1; 
    TD(float r = 1.0f,float max_x2 = 0,float h = 0.001f)
	 :r(r),h(h),max_x2(max_x2)
	{
    
    }

    void Calc(float u);

   private:
};

//调参kp,ki,kd结构体
struct Kpid_t
{
	double kp,ki,kd;
	Kpid_t(double kp = 0,double ki = 0,double kd = 0)
		:kp(kp),ki(ki),kd(kd)
	{}
};

typedef struct
{
    //期望，实际
    double cin,cout,feedback;
    //p,i,d计算
    double p,i,d;
		//Delta,p,i,d计算
    double Dp,Di,Dd;
    //误差
    double last_e,last_last_e,now_e;
	//td跟踪微分器，跟踪误差
	TD td_e;
	//限幅
	double MixI;
    //积分隔离
    float Break_I;
}Pid_t;

class PID
{
private:
  /* data */
public:
	Pid_t pid;
	PID(double Ierror = 0)
	{
		this->pid.Break_I = Ierror;
		this->pid.td_e.r = 100;
	}
	//位置式pid获取
	double GetPidPos(Kpid_t kpid,double cin,double feedback,double max);
	//清除pid
	void clearPID();
	//清除增量
	void PidRstDelta();
};

class FeedForward
{
protected:
	// 输出
	float k;
	// 输出限幅
	float max_cout;

public:
	float cout;

	FeedForward(float max_cout, float k) : max_cout(max_cout), k(k), cout(0) {}
	double GetCout() { return cout; }
};

class FeedTar : public FeedForward
{
private:
	// 上一次目标
	float last_target;
	// 目标误差
	float target_e;

public:
	FeedTar(float max_cout, float k) : FeedForward(max_cout, k), last_target(0), target_e(0) {}

	double UpData(float feedback);
};

class FeedRotating : public FeedForward
{
public:
	FeedRotating(float max_cout, float k) : FeedForward(max_cout, k) {}

	double UpData(float feedback);
};