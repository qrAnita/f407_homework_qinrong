#include "threads.hpp"
#include "M2006.hpp"
#include "DJIMotorHandler.hpp"
#include "can.h"

static M2006 motor;                              ///< 被控电机实例
static DJIMotorHandler *handler = DJIMotorHandler::Instance();

/**
 * @brief control线程：单电机位置-速度双环PID控制
 * @note 【上机前修改】电机所接的CAN总线与ID
 */
void control_thread_entry(ULONG param)
{
    (void)param;

    handler->registerMotor(&motor, &hcan1, 0x201); // 按实际接线修改总线/ID

    motor.controlMode = M2006::POS_MODE;
    motor.positionSet = 1.5707963f; // 目标位置90°（π/2 rad），可改为速度模式测试

    for (;;)
    {
        motor.setOutput();                // 双环PID计算，更新currentSet
        handler->sendControlData();       // 打包并通过CAN发送电流指令

        tx_semaphore_put(&g_control_alive_sem); // 报平安
        tx_thread_sleep(1);                     // 10ms控制周期
    }
}