#ifndef THREADS_HPP
#define THREADS_HPP

#include "tx_api.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct imu_shared_data_t
 * @brief imu线程产出、其他线程消费的共享数据
 */
typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temperature;
} imu_shared_data_t;

extern volatile imu_shared_data_t g_imu_data;    ///< IMU共享数据（imu线程写，其他线程读）
extern volatile uint8_t g_system_init_ok;        ///< BMI088初始化成功后置1（初始化监控）

/* 线程存活信号量：imu/control线程每轮循环 put 一次，alive线程周期 get 判断 */
extern TX_SEMAPHORE g_imu_alive_sem;
extern TX_SEMAPHORE g_control_alive_sem;

/**
 * @brief 创建并启动 alive / imu / control 三个应用线程
 * @note 在 App_ThreadX_Init 中调用
 * @return TX_SUCCESS 成功
 */
UINT Threads_Init(void);

#ifdef __cplusplus
}
#endif

#endif // THREADS_HPP