#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "can.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void);

/**
 * @brief 发送CAN数据。
 * @param hcan 指向CAN句柄的指针，用于配置CAN传输。
 * @param StdId CAN消息的标准标识符
 * @param msg 发送的数据
 * @param len 数据长度
 * @note 该函数用于发送CAN数据，目前只支持标准帧
 */
void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len);
/**
 * @brief CAN 报文分发回调（弱符号，上层模块可重写）。
 * @param hcan 接收报文的总线句柄（&hcan1 或 &hcan2）
 * @param rx_header 报文帧头（含标准帧 ID、DLC 等）
 * @param rx_data 报文数据（最长 8 字节）
 * @note 重写此函数，根据总线和标准帧 ID 分发电机反馈等数据。
 */
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

#ifdef __cplusplus
}
#endif
#endif // BSP_CAN_HPP
