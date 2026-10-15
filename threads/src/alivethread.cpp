#include "threads.hpp"
#include "led.hpp"

/*----------------------共享变量与信号量定义----------------------*/
volatile imu_shared_data_t g_imu_data = {0.0f};
volatile uint8_t g_system_init_ok = 0;
TX_SEMAPHORE g_imu_alive_sem;
TX_SEMAPHORE g_control_alive_sem;

/*----------------------线程控制块与栈----------------------*/
static TX_THREAD alive_thread;
static TX_THREAD imu_thread;
static TX_THREAD control_thread;
static UCHAR alive_stack[1024];
static UCHAR imu_stack[2048];
static UCHAR control_stack[2048];

/* 线程入口（定义在各自文件中） */
static void alive_thread_entry(ULONG param);
extern void imu_thread_entry(ULONG param);
extern void control_thread_entry(ULONG param);

/**
 * @brief 判断某线程在监控窗口内是否存活
 * @param sem 该线程每轮循环 put 的信号量
 * @return true=存活，false=死亡或卡死
 * @note 等待200个tick（按10ms tick约2秒）等不到 put 则判死；
 *       成功后清空计数，避免信号量无限累积。
 */
static bool thread_alive(TX_SEMAPHORE *sem)
{
    if (tx_semaphore_get(sem, 200) != TX_SUCCESS)
    {
        return false;
    }
    while (tx_semaphore_get(sem, TX_NO_WAIT) == TX_SUCCESS)
    {
        // 清空窗口内累积的计数
    }
    return true;
}

/**
 * @brief alive线程：监控初始化结果与imu/control线程存活状态，用LED灯效反映
 * @note 灯效约定：
 *       红色常亮        = 初始化未成功（BMI088自检失败）
 *       红色闪烁        = 有线程死亡（imu或control）
 *       绿色呼吸/闪烁   = 一切正常
 */
static void alive_thread_entry(ULONG param)
{
    (void)param;
    Led::Init();

    for (;;)
    {
        bool imu_ok = thread_alive(&g_imu_alive_sem);
        bool ctl_ok = thread_alive(&g_control_alive_sem);

        if (!g_system_init_ok)
        {
            Led::R_On();
            Led::G_Off();
            Led::B_Off();
        }
        else if (!imu_ok || !ctl_ok)
        {
            /* 任一被监控线程死亡：红色快闪告警 */
            Led::R_Toggle();
            Led::G_Off();
            Led::B_Off();
        }
        else
        {
            /* 全部正常：绿色慢闪心跳 */
            Led::R_Off();
            Led::G_Toggle();
            Led::B_Off();
        }

        tx_thread_sleep(50); // 灯效刷新周期（约0.5s@10ms tick）
    }
}

/**
 * @brief 创建信号量并启动三个应用线程
 * @note 数字越小优先级越高：control > imu > alive
 */
extern "C" UINT Threads_Init(void)
{
    tx_semaphore_create(&g_imu_alive_sem, "imu alive", 0);
    tx_semaphore_create(&g_control_alive_sem, "ctrl alive", 0);

    tx_thread_create(&alive_thread, "alive",
                     alive_thread_entry, 0,
                     alive_stack, sizeof(alive_stack),
                     30, 30, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&imu_thread, "imu",
                     imu_thread_entry, 0,
                     imu_stack, sizeof(imu_stack),
                     15, 15, TX_NO_TIME_SLICE, TX_AUTO_START);

    tx_thread_create(&control_thread, "control",
                     control_thread_entry, 0,
                     control_stack, sizeof(control_stack),
                     10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);

    return TX_SUCCESS;
}