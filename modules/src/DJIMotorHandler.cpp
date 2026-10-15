#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"
#define DJI_MOTOR_ECD_RANGE 8192.0f                 ///< 编码器单圈数值 0~8191
#define DJI_MOTOR_CURRENT_FULL_SCALE 16384.0f       ///< 电流指令满量程
#define DJI_M2006_MAX_CURRENT_AMP 10.0f             ///< M2006 电调满量程对应电流(A)
#define DJI_M2006_REDUCTION_RATIO 36.0f             ///< M2006 减速比 36:1
#define DJI_MOTOR_TEMP_SCALE 1.0f                   ///< 温度 1℃/LSB

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void CAN_Rx_Callback(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (rx_header->StdId >= 0x201 && rx_header->StdId <= 0x208)
    {
        DJIMotorHandler::Instance()->updateFeedback(hcan, rx_data, rx_header->StdId - 0x201);
    }
}

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    // TODO: 校验 CAN 句柄和 0x201~0x208 ID，再登记到对应总线与槽位。
    // TODO: 按 ID 设置对应的 0x200/0x1FF 报文存在标志。
     if (motor == nullptr || hcan == nullptr || canId < 0x201 || canId > 0x208)
    {
        return;
    }

    uint8_t bus = (hcan == &hcan1) ? 0 : 1; // 总线索引：CAN1=0, CAN2=1
    uint8_t slot = canId - 0x201;           // 槽位：0~7

    if (DJIMotorList[bus][slot] != nullptr)
    {
        return; // 该槽位已注册
    }

    // 登记电机并回填总线与 ID
    DJIMotorList[bus][slot] = motor;
    motor->hcan = hcan;
    motor->canId = canId;

    // 按 ID 设置 0x200 / 0x1FF 控制报文存在标志
    if (bus == 0)
    {
        if (slot < 4)
            CAN1_0x200_Exist = true;
        else
            CAN1_0x1FF_Exist = true;
    }
    else
    {
        if (slot < 4)
            CAN2_0x200_Exist = true;
        else
            CAN2_0x1FF_Exist = true;
    }
}

void DJIMotorHandler::sendControlData()
{
    for (uint8_t bus = 0; bus < 2; bus++)
    {
        uint8_t *frame_0x200 = (bus == 0) ? can1_send_data_0 : can2_send_data_0;
        uint8_t *frame_0x1FF = (bus == 0) ? can1_send_data_1 : can2_send_data_1;
        bool exist_0x200 = (bus == 0) ? CAN1_0x200_Exist : CAN2_0x200_Exist;
        bool exist_0x1FF = (bus == 0) ? CAN1_0x1FF_Exist : CAN2_0x1FF_Exist;
        CAN_HandleTypeDef *hcan = (bus == 0) ? &hcan1 : &hcan2;

        for (uint8_t slot = 0; slot < 8; slot++)
        {
            DJIMotor *motor = DJIMotorList[bus][slot];
            if (motor == nullptr)
            {
                continue;
            }

            uint8_t *frame = (slot < 4) ? frame_0x200 : frame_0x1FF;
            uint8_t offset = (slot % 4) * 2; // 每电机占 2 字节，大端
            int16_t current = motor->currentSet;

            int32_t frame_current = (int32_t)motor->currentSet * (int32_t)DJI_MOTOR_CURRENT_FULL_SCALE
                                    / (int32_t)(DJI_M2006_MAX_CURRENT_AMP * 1000.0f);
            if (frame_current > (int32_t)DJI_MOTOR_CURRENT_FULL_SCALE - 1)
                frame_current = (int32_t)DJI_MOTOR_CURRENT_FULL_SCALE - 1;
            if (frame_current < -((int32_t)DJI_MOTOR_CURRENT_FULL_SCALE - 1))
                frame_current = -((int32_t)DJI_MOTOR_CURRENT_FULL_SCALE - 1);

            frame[offset] = (uint8_t)((frame_current >> 8) & 0xFF);
            frame[offset + 1] = (uint8_t)(frame_current & 0xFF);
        }
        

        if (exist_0x200)
            CAN_Transmit(hcan, 0x200, frame_0x200, 8);
        if (exist_0x1FF)
            CAN_Transmit(hcan, 0x1FF, frame_0x1FF, 8);
    }
    // TODO: 将各电机 currentSet 按大端序放入正确的 8 字节控制帧。
    // TODO: 仅发送包含已注册电机的报文；未完成前不发送 CAN 电流命令。
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    // TODO: 校验总线、反馈数据和索引，再找到对应电机并调用 UpdateSensorData。
   if (rx_data == nullptr || index < 0 || index > 7)
    {
        return;
    }

    uint8_t bus = (hcan == &hcan1) ? 0 : 1;
    DJIMotor *motor = DJIMotorList[bus][index];
    if (motor == nullptr)
    {
        return;
    }

    UpdateSensorData(motor, rx_data);
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    // TODO: 从 CAN 数据解析编码器、转速、电流和温度。
    // TODO: 处理编码器回绕，并按减速比换算输出轴位置和角速度。
    motor->motorFeedback.last_ecd = motor->motorFeedback.ecd;
    motor->motorFeedback.lastSpeedFdb = motor->motorFeedback.speedFdb;
    motor->motorFeedback.lastPositionFdb = motor->motorFeedback.positionFdb;

    motor->motorFeedback.ecd = (uint16_t)((can_data[0] << 8) | can_data[1]);
    motor->motorFeedback.speed_rpm = (int16_t)((can_data[2] << 8) | can_data[3]);

    int16_t current_raw = (int16_t)((can_data[4] << 8) | can_data[5]);
    motor->motorFeedback.currentFdb = current_raw * (DJI_M2006_MAX_CURRENT_AMP / DJI_MOTOR_CURRENT_FULL_SCALE);
    motor->motorFeedback.temperatureFdb = can_data[6] * DJI_MOTOR_TEMP_SCALE;

    // 编码器回绕处理：增量超出半圈则反向
    int32_t delta = (int32_t)motor->motorFeedback.ecd - motor->motorFeedback.last_ecd;
    if (delta > (int32_t)(DJI_MOTOR_ECD_RANGE / 2))
        delta -= (int32_t)DJI_MOTOR_ECD_RANGE;
    if (delta < -(int32_t)(DJI_MOTOR_ECD_RANGE / 2))
        delta += (int32_t)DJI_MOTOR_ECD_RANGE;

    // 电机轴角速度(rad/s) → 输出轴角速度(rad/s)
    motor->motorFeedback.speedFdb = motor->motorFeedback.speed_rpm * (2.0f * 3.1415926f / 60.0f) / DJI_M2006_REDUCTION_RATIO;

    // 输出轴位置(rad)累计
    motor->motorFeedback.positionFdb += delta * (2.0f * 3.1415926f / DJI_MOTOR_ECD_RANGE) / DJI_M2006_REDUCTION_RATIO;

    motor->AliveFlag++; // 收到一次反馈，存活计数加一
}


void DJIMotorHandler::AllMotorAliveCheck()
{
    for (uint8_t bus = 0; bus < 2; bus++)
    {
        for (uint8_t slot = 0; slot < 8; slot++)
        {
            if (DJIMotorList[bus][slot] != nullptr)
            {
                DJIMotorList[bus][slot]->AliveCheck();
            }
        }
    }
    // TODO: 遍历已注册电机并调用 AliveCheck。
}
