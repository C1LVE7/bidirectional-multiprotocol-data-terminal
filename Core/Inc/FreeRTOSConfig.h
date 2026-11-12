/* FreeRTOS 11.3.0 配置文件，适用于 STM32F407VET6、Keil/RVDS Cortex-M4F 移植层。 */
/* 芯片主频来自本工程时钟树：8 MHz 外部晶振经 PLL 倍频至 168 MHz。 */
/* NVIC 采用 4 位抢占优先级；数值越小，中断优先级越高。 */

#ifndef FREERTOS_CONFIG_H /* 防止本配置文件被重复包含。 */
#define FREERTOS_CONFIG_H /* 定义 FreeRTOS 配置文件保护宏。 */

/*============================== 硬件参数 ==============================*/
#define configCPU_CLOCK_HZ                          ( 168000000UL ) /* CPU 内核时钟为 168 MHz。 */
#define configPRIO_BITS                             4 /* STM32F407 实现 4 个 NVIC 优先级位。 */

/*============================== 调度配置 ==============================*/
#define configUSE_PREEMPTION                        1 /* 启用抢占式任务调度。 */
#define configUSE_TIME_SLICING                      1 /* 同优先级就绪任务使用时间片轮转。 */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1 /* 使用 Cortex-M4 前导零指令优化最高优先级选择。 */
#define configUSE_TICKLESS_IDLE                     0 /* 暂不启用无节拍低功耗，避免引入额外时钟补偿条件。 */
#define configTICK_RATE_HZ                          ( 1000U ) /* 系统节拍频率为 1000 Hz，即每个 Tick 为 1 ms。 */
#define configTICK_TYPE_WIDTH_IN_BITS               TICK_TYPE_WIDTH_32_BITS /* Tick 计数使用 32 位，1 kHz 下约 49.7 天回绕。 */
#define configMAX_PRIORITIES                        8U /* 提供 0 至 7 共 8 个任务优先级。 */
#define configMINIMAL_STACK_SIZE                    128U /* 空闲任务最小栈为 128 个 32 位字，即 512 字节。 */
#define configMAX_TASK_NAME_LEN                     16U /* 任务名最多保存 15 个字符和结尾空字符。 */
#define configIDLE_SHOULD_YIELD                     1 /* 空闲任务向同优先级的用户任务主动让出处理器。 */
#define configTASK_NOTIFICATION_ARRAY_ENTRIES       1U /* 每个任务配置 1 个直接任务通知槽。 */
#define configQUEUE_REGISTRY_SIZE                   8U /* 调试器最多登记并显示 8 个队列或信号量。 */
#define configENABLE_BACKWARD_COMPATIBILITY         1 /* 保留旧版 FreeRTOS API 名称兼容层。 */
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS     0U /* 不为任务分配线程局部存储指针。 */
#define configSTACK_DEPTH_TYPE                      size_t /* 栈深度参数使用 size_t 类型。 */
#define configMESSAGE_BUFFER_LENGTH_TYPE            size_t /* 消息缓冲区长度使用 size_t 类型。 */
#define configUSE_NEWLIB_REENTRANT                  0 /* 不为每个任务分配 Newlib 重入结构。 */
#define configUSE_MINI_LIST_ITEM                    1 /* 使用精简列表项以节省内核 RAM。 */

/*============================ 软件定时器 ==============================*/
#define configUSE_TIMERS                            1 /* 启用 FreeRTOS 软件定时器。 */
#define configTIMER_TASK_PRIORITY                   ( configMAX_PRIORITIES - 1U ) /* 定时器服务任务使用最高任务优先级 7。 */
#define configTIMER_TASK_STACK_DEPTH                256U /* 定时器服务任务栈为 256 个 32 位字，即 1 KiB。 */
#define configTIMER_QUEUE_LENGTH                    10U /* 定时器命令队列最多容纳 10 条命令。 */

/*============================ 内存分配 ================================*/
#define configSUPPORT_STATIC_ALLOCATION             1 /* 允许使用静态方式创建任务和内核对象。 */
#define configSUPPORT_DYNAMIC_ALLOCATION            1 /* 允许 heap_4 使用动态方式创建任务和内核对象。 */
#define configTOTAL_HEAP_SIZE                       ( 32U * 1024U ) /* 为 heap_4 分配 32 KiB 内核堆。 */
#define configAPPLICATION_ALLOCATED_HEAP            0 /* 由 heap_4.c 定义 ucHeap，应用无需另行提供数组。 */
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP   0 /* 任务栈与其他对象共用 FreeRTOS 内核堆。 */
#define configKERNEL_PROVIDED_STATIC_MEMORY         1 /* 由 FreeRTOS 提供空闲任务和定时器任务的静态内存。 */

/*============================ 中断优先级 ==============================*/
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     15 /* HAL 数值 15 是 STM32F407 最低可编程中断优先级。 */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5 /* HAL 数值 5 是可调用 FromISR API 的最高优先级边界。 */
#define configKERNEL_INTERRUPT_PRIORITY             ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) ) /* 内核中断寄存器优先级为 0xF0。 */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) ) /* BASEPRI 屏蔽阈值为 0x50。 */
#define configMAX_API_CALL_INTERRUPT_PRIORITY       configMAX_SYSCALL_INTERRUPT_PRIORITY /* 使用新版宏名表示同一个 FromISR 调用边界。 */

/*============================ 钩子与检查 ==============================*/
#define configUSE_IDLE_HOOK                         0 /* 不调用应用层空闲任务钩子。 */
#define configUSE_TICK_HOOK                         0 /* 不调用应用层系统节拍钩子。 */
#define configUSE_MALLOC_FAILED_HOOK                1 /* 内存申请失败钩子 */
#define configUSE_DAEMON_TASK_STARTUP_HOOK          0 /* 不调用定时器服务任务启动钩子。 */
#define configCHECK_FOR_STACK_OVERFLOW              2 /* 栈溢出钩子*/
#define configCHECK_HANDLER_INSTALLATION            1 /* 启动调度器时检查 SVC 和 PendSV 向量是否安装正确。 */
#define configASSERT( x )                           do { if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ) { } } } while( 0 ) /* 断言失败时关中断并停在原处，便于调试定位。 */

/*============================ 统计与跟踪 ==============================*/
#define configGENERATE_RUN_TIME_STATS               0 /* 不生成任务运行时间统计。 */
#define configUSE_TRACE_FACILITY                    0 /* 不启用内核跟踪结构扩展。 */
#define configUSE_STATS_FORMATTING_FUNCTIONS        0 /* 不编译 vTaskList 等文本格式化统计函数。 */

/*============================ 内核功能 ================================*/
#define configUSE_TASK_NOTIFICATIONS                1 /* 启用轻量级直接任务通知。 */
#define configUSE_MUTEXES                           1 /* 启用互斥量和优先级继承。 */
#define configUSE_RECURSIVE_MUTEXES                 1 /* 启用递归互斥量。 */
#define configUSE_COUNTING_SEMAPHORES               1 /* 启用计数信号量。 */
#define configUSE_QUEUE_SETS                        1 /* 启用队列集。 */
#define configUSE_EVENT_GROUPS                      1 /* 启用事件组。 */
#define configUSE_STREAM_BUFFERS                    1 /* 启用流缓冲区和消息缓冲区。 */
#define configUSE_APPLICATION_TASK_TAG              0 /* 不为任务启用应用标签回调。 */
#define configUSE_POSIX_ERRNO                       0 /* 不为每个任务保存 POSIX errno。 */
#define configUSE_CO_ROUTINES                       0 /* 不启用已不推荐用于新设计的协程功能。 */
#define configMAX_CO_ROUTINE_PRIORITIES             2U /* 若以后启用协程，则提供 2 个协程优先级。 */

/*============================ 可用任务 API ============================*/
#define INCLUDE_vTaskPrioritySet                    1 /* 编译 vTaskPrioritySet API。 */
#define INCLUDE_uxTaskPriorityGet                   1 /* 编译 uxTaskPriorityGet API。 */
#define INCLUDE_vTaskDelete                         1 /* 编译 vTaskDelete API。 */
#define INCLUDE_vTaskSuspend                        1 /* 编译 vTaskSuspend 和 vTaskResume API。 */
#define INCLUDE_xTaskDelayUntil                     1 /* 编译 xTaskDelayUntil 周期延时 API。 */
#define INCLUDE_vTaskDelay                          1 /* 编译 vTaskDelay 相对延时 API。 */
#define INCLUDE_xTaskGetSchedulerState              1 /* 编译 xTaskGetSchedulerState API。 */
#define INCLUDE_xTaskGetCurrentTaskHandle           1 /* 编译 xTaskGetCurrentTaskHandle API。 */
#define INCLUDE_uxTaskGetStackHighWaterMark         1 /* 编译 uxTaskGetStackHighWaterMark API。 */
#define INCLUDE_uxTaskGetStackHighWaterMark2        1 /* 编译返回配置类型的栈余量查询 API。 */
#define INCLUDE_xTaskGetIdleTaskHandle              1 /* 编译 xTaskGetIdleTaskHandle API。 */
#define INCLUDE_eTaskGetState                       1 /* 编译 eTaskGetState API。 */
#define INCLUDE_xTimerPendFunctionCall              1 /* 编译中断或任务挂起函数到定时器任务的 API。 */
#define INCLUDE_xTaskAbortDelay                     1 /* 编译 xTaskAbortDelay API。 */
#define INCLUDE_xTaskGetHandle                      1 /* 编译按任务名查找句柄的 API。 */
#define INCLUDE_xTaskResumeFromISR                  1 /* 编译中断中恢复任务的 API。 */

/*========================== Cortex-M4F 异常映射 =======================*/
#define vPortSVCHandler                             SVC_Handler /* 将 FreeRTOS SVC 处理函数绑定到启动文件向量名。 */
#define xPortPendSVHandler                          PendSV_Handler /* 将 FreeRTOS PendSV 处理函数绑定到启动文件向量名。 */
#define xPortSysTickHandler 						SysTick_Handler

#endif /* 结束 FREERTOS_CONFIG_H 配置文件保护。 */
