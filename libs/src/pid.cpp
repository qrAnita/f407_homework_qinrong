#include "pid.hpp"

PID::PID(float kp, float ki, float kd, float maxOut, float maxIOut, int mode)
    : mode(mode), kp(kp), ki(ki), kd(kd), maxOut(maxOut), maxIOut(maxIOut)
{
    Clear();
}

void PID::Tuning(float tuning_kp, float tuning_ki, float tuning_kd)
{
    kp = tuning_kp;
    ki = tuning_ki;
    kd = tuning_kd;
}
static inline float Limit_Value(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}
void PID::UpdateResult()
{
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    switch (mode)
    {
    case PID_POSITION: // 位置式 PID：u = Kp*e + Ki*∑e + Kd*(e - e_prev)
        pResult = kp * err[0];
        iResult += ki * err[0];                    // 积分累加
        iResult = Limit_Value(iResult, maxIOut);   // 积分限幅（anti-windup）
        dResult = kd * (err[0] - err[1]);
        result = Limit_Value(pResult + iResult + dResult, maxOut); // 输出限幅
        break;

    case PID_DELTA: // 增量式 PID：Δu = Kp*(e-e1) + Ki*e + Kd*(e - 2*e1 + e2)
        pResult = kp * (err[0] - err[1]);
        iResult = ki * err[0];
        iResult = Limit_Value(iResult, maxIOut);   // 积分项限幅
        dResult = kd * (err[0] - 2.0f * err[1] + err[2]);
        result += Limit_Value(pResult + iResult + dResult, maxOut); // 增量累加后输出限幅
        result = Limit_Value(result, maxOut);
        break;

    default: // 未知模式保持零输出
        pResult = iResult = dResult = result = 0.0f;
        break;
    }
}

void PID::Clear()
{
    ref = fdb = 0.0f;
    err[0] = err[1] = err[2] = 0.0f;
    pResult = iResult = dResult = result = 0.0f;
}
