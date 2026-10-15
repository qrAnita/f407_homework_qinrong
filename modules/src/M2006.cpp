#include "M2006.hpp"
#define M2006_REDUCTION_RATIO 36.0f ///< M2006 减速比
#define M2006_MAX_CURRENT_MA 10000  ///< 最大电流限制(mA)，对应电调满量程 10A

#define M2006_SPD_PID_KP 6.0f   ///< 速度环比例
#define M2006_SPD_PID_KI 0.5f   ///< 速度环积分
#define M2006_SPD_PID_KD 0.0f   ///< 速度环微分
#define M2006_SPD_PID_MAX_IOUT 2000.0f ///< 速度环积分限幅

#define M2006_POS_PID_KP 3.0f   ///< 位置环比例
#define M2006_POS_PID_KI 0.0f   ///< 位置环积分
#define M2006_POS_PID_KD 0.0f   ///< 位置环微分
#define M2006_POS_PID_MAX_OUT 20.0f ///< 位置环输出限幅(rad/s)，即最大速度
M2006::M2006()
{
    gearBox = GearBox_M2006;
    maxCurrent = M2006_MAX_CURRENT_MA;

    // 配置速度环 PID（位置式，带积分与输出限幅）
    speedPid.Tuning(M2006_SPD_PID_KP, M2006_SPD_PID_KI, M2006_SPD_PID_KD);
    speedPid.maxOut = (float)M2006_MAX_CURRENT_MA;
    speedPid.maxIOut = M2006_SPD_PID_MAX_IOUT;

    // 配置位置环 PID，输出作为速度环目标
    positionPid.Tuning(M2006_POS_PID_KP, M2006_POS_PID_KI, M2006_POS_PID_KD);
    positionPid.maxOut = M2006_POS_PID_MAX_OUT;
    positionPid.maxIOut = 0.0f;

    controlMode = RELAX_MODE; // 完成配置前保持松开，不允许驱动电机
    // TODO: 根据 M2006 手册设置减速比、允许电流和速度/位置环 PID 参数。
    // 基类初始化为松开模式、零电流；完成配置前不允许驱动电机。
}

void M2006::setOutput()
{
    // TODO: 分别实现松开、速度和位置模式；位置模式需要串联位置环与速度环。
    // TODO: 将 PID 输出限制在 maxCurrent 范围内后写入 currentSet。
   switch (controlMode)
    {
    case RELAX_MODE: // 松开模式：零输出
        currentSet = 0;
        break;

    case SPD_MODE: // 速度模式：单速度环
        speedPid.ref = speedSet;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        currentSet = (int16_t)speedPid.result;
        break;

    case POS_MODE: // 位置模式：位置环(输出目标速度)串联速度环
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();

        speedPid.ref = positionPid.result; // 外环输出即内环目标
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        currentSet = (int16_t)speedPid.result;
        break;

    default:
        currentSet = 0;
        break;
    }

    // 最终输出限制在允许电流范围内
    if (currentSet > (int16_t)maxCurrent)
        currentSet = (int16_t)maxCurrent;
    if (currentSet < -(int16_t)maxCurrent)
        currentSet = -(int16_t)maxCurrent;
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    // TODO: 比较 AliveFlag 与 Pre_AliveFlag，更新在线状态并保存本次计数。
   if (MotorState == MOTOR_OFFLINE && AliveFlag != Pre_AliveFlag)
    {
        MotorState = MOTOR_ONLINE;
    }
    else if (MotorState == MOTOR_ONLINE && AliveFlag == Pre_AliveFlag)
    {
        MotorState = MOTOR_OFFLINE;
    }
    Pre_AliveFlag = AliveFlag;

    return MotorState;
}
