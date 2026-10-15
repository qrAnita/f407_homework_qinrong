#include "threads.hpp"
#include "BMI088.hpp"
#include "IMU.hpp"

/* 温度控制开关：若 CubeMX 未使能 TIM10（htim10 链接报错），
 * 注释掉本行即可关闭加热温控功能 */
#define USE_IMU_TEMP_CONTROL

static BMI088::cBMI088 bmi088; ///< BMI088 实例

/**
 * @brief imu线程：BMI088初始化 → 循环读取数据 → 写入共享变量
 */
void imu_thread_entry(ULONG param)
{
    (void)param;
    acc_data_t acc;
    gyro_data_t gyro;
    float temp;

    /* 初始化：配置 → 校验芯片ID → 零偏标定 */
    bmi088.Config();
    bmi088.VerifyAccChipID();
    bmi088.VerifyGyroChipID();

    if (!bmi088.self_test.INIT_ERR &&
        !bmi088.self_test.ACC_CHIP_ID_ERR &&
        !bmi088.self_test.GYRO_CHIP_ID_ERR)
    {
        bmi088.Calibrate();
        g_system_init_ok = 1; // 通知alive线程：初始化成功
    }

    for (;;)
    {
        bmi088.ReadAccData(&acc);
        bmi088.ReadGyroData(&gyro);
        bmi088.ReadAccTemperature(&temp);

        g_imu_data.acc_x = acc.x;
        g_imu_data.acc_y = acc.y;
        g_imu_data.acc_z = acc.z;
        g_imu_data.gyro_x = gyro.x;
        g_imu_data.gyro_y = gyro.y;
        g_imu_data.gyro_z = gyro.z;
        g_imu_data.temperature = temp;

#ifdef USE_IMU_TEMP_CONTROL
        bmi088.TemperatureControl(40.0f); // 陀螺仪恒温40℃
#endif

        tx_semaphore_put(&g_imu_alive_sem); // 报平安
        tx_thread_sleep(10);                // 100ms采样周期
    }
}