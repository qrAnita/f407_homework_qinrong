#include "bsp_can.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 初始化CAN滤波器配置。
 * 设置CAN硬件的滤波器，用于优化接收数据的处理。
 * 更多信息，请参考原文，链接：https://blog.csdn.net/weixin_54448108/article/details/128570593
 */
void CAN_Init(void)
{
    CAN_FilterTypeDef can_filter_st;                   ///< 定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;           ///< ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;  ///< 设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; ///< 过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;               ///< ID高位
    can_filter_st.FilterIdLow = 0x0000;                ///< ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;           ///< 过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;            ///< 过滤器掩码低位

    can_filter_st.FilterBank = 0;                                      ///< 过滤器组-双CAN可指定0~27
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                 ///< 与过滤器组管理的 FIFO
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan1);                                             ///< 使能CAN1控制器
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断

    can_filter_st.SlaveStartFilterBank = 14;                           ///< 双CAN模式下规定CAN的主从模式的过滤器分配，从过滤器为14
    can_filter_st.FilterBank = 14;                                     ///< 过滤器组-双CAN可指定0~27
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);                      ///< HAL库配置过滤器函数
    HAL_CAN_Start(&hcan2);                                             ///< 使能CAN2控制器
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); ///< 使能CAN的各种中断
}

void CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Id, uint8_t *msg, uint16_t len)
{
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        // 如果邮箱已满，可以选择等待或者采取其他操作
        // 例如，可以添加一个超时机制或者直接返回错误状态
        // Monitor::instance()->Log_Messages(Monitor::WARNING, (uint8_t *)"CAN Tx Mailbox is full\r\n");
        return;
    }
        // 避免在未初始化时发送数据：HAL_CAN_Start 之后状态不再是 RESET
    if (HAL_CAN_GetState(hcan) == HAL_CAN_STATE_RESET)
    {
        return;
    }

    // 校验标准帧 ID（11 位）和数据长度 DLC（0~8）
    if (Id > 0x7FF || len > 8)
    {
        return;
    }

    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        // 如果邮箱已满，可以选择等待或者采取其他操作
        // 例如，可以添加一个超时机制或者直接返回错误状态
        // Monitor::instance()->Log_Messages(Monitor::WARNING, (uint8_t *)"CAN Tx Mailbox is full\r\n");
        return;
    }

    // 构造标准数据帧帧头
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = Id;                 ///< 标准帧 ID
    tx_header.IDE = CAN_ID_STD;           ///< 标准帧
    tx_header.RTR = CAN_RTR_DATA;         ///< 数据帧
    tx_header.DLC = len;                  ///< 数据长度 0~8
    tx_header.TransmitGlobalTime = DISABLE;

    uint32_t tx_mailbox = 0;
    HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &tx_mailbox);    // TODO: 校验标准帧 ID 和 DLC (0~8)，构造帧头并调用 HAL_CAN_AddTxMessage。
    (void)Id;
    (void)msg;
    (void)len;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    // 只处理标准帧
    if (rx_header.IDE != CAN_ID_STD)
    {
        return;
    }

    // hcan == &hcan1 表示 CAN1 总线，hcan == &hcan2 表示 CAN2 总线；
    // rx_header.StdId 为反馈帧 ID，由上层模块重写 CAN_Rx_Callback 解析。
    CAN_Rx_Callback(hcan, &rx_header, rx_data);
}

/**
 * @brief CAN 报文分发函数（弱符号，可被上层模块重写）。
 * 电机等模块重写本函数，根据 hcan（区分总线）和 StdId 解算反馈数据。
 */
__weak void CAN_Rx_Callback(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    (void)hcan;
    (void)rx_header;
    (void)rx_data;
}// TODO: 实现 CAN 接收回调，根据总线和标准帧 ID 分发电机反馈。
