# 双向多协议数据终端

基于 STM32F407VET6 + FreeRTOS 的多协议数据终端：采集现场温湿度本地数显、实时上云，云端可下发命令反控本地显示，形成**上行遥测 → 下行命令 → 执行回执**的完整闭环。

技术栈：FreeRTOS-Kernel V11.3.0 / coreMQTT v5.0.2 / Modbus RTU 主站（手写） / ESP8266 AT 透传 / AHT20（I2C） / RS485 六位数显表。

```text
        AHT20 ──I2C1──┐
                      ├─► 采集任务 ──互斥量快照──► MQTT 任务 ──USART3──► ESP8266 ──► Broker
  六位数显表 ◄─RS485──┘         ▲                    │                              │
        ▲                       │                    │  command                     │
        └──── FC06 写显示 ◄── 显示任务 ◄──volatile───┘  ◄───────────────────────────┘
                                  └──── 队列回执 ───► MQTT 任务 ──► result 主题
```

---

## 1. 多任务架构

四个任务，优先级按实时性排序；每个任务创建时逐个判 `pdPASS`，不用 `||` 短路，任一失败即终止启动：

| 任务 | 优先级 | 栈（字） | 周期 | 职责 |
|---|---|---|---|---|
| MQTT | 4 | 768 | 事件驱动 + 5 s 遥测 | 连接维持、订阅命令、上报遥测与回执 |
| 显示 | 3 | 384 | 1.5 s 写 / 3 s 轮显 | Modbus FC06 刷新数显表、消费回执队列 |
| 采集 | 2 | 256 | 2 s | AHT20 读温湿度，写共享快照 |
| 心跳 | 1 | 128 | 500 ms | LED1 翻转；LED2 由网络状态决定 |

栈大小经 `vTaskList` 实测剩余水位核对，留约 30% 余量。

**三类 IPC 按数据语义选型**，不是随手挑的：

| 共享数据 | 语义 | 选型 | 理由 |
|---|---|---|---|
| 温湿度快照 `g_telemetry` | 多字段、需一致读 | 互斥量 `g_tele_mutex` | 防止读到"温度新、湿度旧"的撕裂快照 |
| 显示模式 `g_disp_mode` | 单字节、只关心最新值 | `volatile` | 丢失中间值无害，加锁反而阻塞命令回调 |
| 执行回执 | 每次都要送达、不能丢 | 队列（深度 4） | 断线期间回执留在队列里，重连后补发 |

**双钩子兜底**：`vApplicationStackOverflowHook` 与 `vApplicationMallocFailedHook` 记录故障任务名与剩余堆信息，然后 `while(1)` 停死——钩子内不调任何 FreeRTOS API、不用日志（可能正持有锁）。

---

## 2. 硬件与接线

| 项目 | 配置 |
|---|---|
| 主控 | STM32F407VETx（LQFP100） |
| 时钟 | HSE 8 MHz → PLLM 8 / PLLN 336 / PLLP 2 → 168 MHz；APB1 42 / APB2 84 MHz |
| HAL 时基 | TIM4（1 ms），SysTick 留给 FreeRTOS |
| 优化等级 | `-O2 -Ospace`（`<Optim>3`） |

| 功能 | 引脚 | 参数 | 备注 |
|---|---|---|---|
| USART1 日志 | PA9 / PA10 | 115200-8-N-1 | 轮询发送，不开中断 |
| USART2 RS485 | PA2 / PA3 | 9600-8-N-1 | 阻塞轮询，未开中断 |
| USART3 ESP8266 | PB10 / PB11 | 115200-8-N-1 | 手写 ISR + 512 B 环形缓冲 |
| I2C1 AHT20 | PB6 / PB7 | 100 kHz，7 位地址 | AF 开漏 |
| RS485 方向 | PA1 | 推挽输出 | 高=发送，低=接收（上电默认接收） |
| ESP8266 复位 | PB8 | 推挽输出 | 初始 High，异常时拉低 200 ms 硬复位 |
| LED1 心跳 / LED2 网络 | PA6 / PA7 | 推挽，低电平点亮 | 初始熄灭 |

**板载引脚冲突（按原理图核实，接线前必看）**：

| 引脚 | 板上已连 | 后果 |
|---|---|---|
| PB6 / PB7 | NRF24L01 的 CE / CS | 插上 NRF 模块则 I2C1 失效，AHT20 读不到 |
| PB8 | NRF24L01 的 IRQ | 插上 NRF 模块则 ESP8266 无法硬复位 |
| PB0 / PB3 / PB4 / PB5 | W25Q16 Flash | 不可占用 |

---

## 3. 使用

### 3.1 改配置

全部参数集中在 `APP/app_config.h`，上机前至少要改这三项：

```c
#define WIFI_SSID       "你的WiFi名"      // 仅支持 2.4 GHz
#define WIFI_PASSWORD   "你的WiFi密码"
#define MQTT_BROKER_HOST "x.x.x.x"        // 只能填 IP：透传走 AT+CIPSTART，不做域名解析
```

`broker.emqx.io` 的 IP 会变，连不上先 `nslookup broker.emqx.io` 取当前地址。多台设备连同一 Broker 必须改 `MQTT_CLIENT_ID`，否则会被互相踢下线。

### 3.2 编译与烧录

1. Keil 打开 `MDK-ARM/project.uvprojx`（需 `STM32F4xx_DFP` 器件包）
2. `Project → Build Target`（F7），固件输出 `MDK-ARM/project/project.hex`
3. ST-Link / J-Link 下载（SWD：PA13 / PA14）
4. 串口 USART1（115200）观察启动日志

参考构建结果：`0 Error / 0 Warning`，`Code 65294 / RO 6922 / RW 204 / ZI 39012`。

### 3.3 上电自检顺序

1. 日志出现内核与外设初始化信息
2. LED1 每 500 ms 翻转（心跳任务已跑起来）
3. ESP8266 完成硬复位 → 连 AP → 建 TCP 透传
4. MQTT CONNECT 成功，发布上线状态，LED2 常亮
5. 每 5 s 在 `thrms/001/telemetry` 收到一条温湿度
6. 数显表按当前模式刷新（默认待机）

### 3.4 云端反控

用 MQTTX 连同一 Broker，向 `thrms/001/command` 发布：

```json
{"display":1}
```

取值：`0` 待机、`1` 固定显示温度、`2` 固定显示湿度、`3` 温湿轮显（每 3 s 切换）。

执行结果从 `thrms/001/result` 回来：

```json
{"display":"1","result":"ok"}
```

`result` 为 `fail` 表示这次写从机没成功，上层会在下一轮重试。

### 3.5 四个 MQTT 主题

| 主题 | 方向 | 载荷 |
|---|---|---|
| `thrms/001/telemetry` | 上行，5 s | 温度、湿度、`sensor_valid` |
| `thrms/001/command` | 下行 | `{"display":0~3}` |
| `thrms/001/result` | 上行 | `{"display":"x","result":"ok/fail"}` |
| `thrms/001/status` | 上行 | 在线状态；掉线由 LWT 遗嘱代发 |

---

## 4. 通信实现

### 4.1 MQTT 参数与判死链

| 项 | 取值 | 原因 |
|---|---|---|
| 遥测 QoS | 0 | 5 s 一轮，丢包由下轮数据自然覆盖，不值得为它做重传 |
| KeepAlive | 30 s | Broker 侧两倍容差 |
| PING 触发 | RX 空闲 10 s 发 PINGREQ | 有数据往来时不必发心跳 |
| PINGRESP 超时 | 5 s | 与上一项合计约 **15 s 判死** |
| 重连退避 | 2 → 4 → 8 → 16 → 30 s 封顶 | 固定 1 s 快连会被 Broker 以 `0x8A` 限流封禁 |
| LWT 遗嘱 | 已注册 | 拔电、断网等非正常掉线由 Broker 代发离线通知 |

**协议栈与硬件解耦**：coreMQTT 通过 `transport_send` / `transport_recv` 两个函数指针访问网络，底层换成什么模块都不影响协议层。

### 4.2 Modbus RTU 主站（手写，零第三方库）

- **CRC-16** 用 `0xA001` 反射多项式
- **变长帧**按功能码判长度，不按固定字节数收
- **发完等 TC 而不是 TXE** 再切回接收：TXE 只表示发送寄存器空，此时最后一个字节还在移位寄存器里，切早了会把帧尾 CRC 截断
- **回显逐字节核对**：从机回显必须与请求一致，杜绝"从机没应答、主站却认为成功"
- **只用 FC06** 写 `0x0000`（数显表的 F0-00 显示缓存区）。FC03 已实现但无调用者，链接器按 `Removing` 丢弃（记 224 B，未进镜像）——这是设计如此，不是缺失
- 时序：从机应答延迟 30 ms，主站轮询节拍 1.5 s（须小于从机超时，面板设 ≥ 2 s，否则报 `E.Con1`）

### 4.3 环形缓冲与 JSON

- **USART3 接收**：512 B 单生产者单消费者环形缓冲，ISR 只写 `head`、任务只写 `tail`，索引分离因而免锁
- **JSON 解析**：手写严格状态机，不是字符串查找。逐字符走状态机、校验字段名与取值范围，非法载荷直接丢弃，不作猜测

### 4.4 AHT20 采集

20 bit 原始值 → CRC-8 校验 → 定点换算，**全程整数无浮点**，避免引入软浮点库。读失败时置 `sensor_valid = 0` 但**保留上次温湿度**，云端看到的是"旧值 + valid:0"，而不是突然跳零。

---

## 5. 异常分级处置

| 级别 | 异常 | 处置 |
|---|---|---|
| 通信 | Modbus 写失败 | 返回错误码，上层重试并回执 `fail` |
| 通信 | MQTT 断线 | 15 s 判死 + 指数退避重连，回执在队列里等重连后补发 |
| 外设 | AHT20 读失败 | 保留旧值，只清 `sensor_valid` |
| 外设 | ESP8266 异常 | 拉低 RST 200 ms 硬复位 + 重连 AP，自愈无需人工 |
| 系统 | 栈溢出 / 内存申请失败 | 双钩子记录故障任务与剩余堆，然后停死 |
| 启动 | 外设初始化失败 | 只记日志不阻断启动，由后续自愈机制兜底 |

---

## 6. 资源占用

| 项 | 数值 | 占比 |
|---|---|---|
| 自研代码 | 16 个文件 / 1959 行（有效代码 1107 行） | — |
| Flash（Code + RO = 72216 B） | 70.5 KB / 512 KB | 13.8% |
| 静态 RAM（RW + ZI = 39216 B） | 38.3 KB / 128 KB | 29.9% |
| 其中内核堆 / 四任务栈 | 32 KB / 6 KB | — |

---

## 7. 目录结构

```text
├── APP/                     应用层
│   ├── app_config.h         全部可调参数（WiFi、Broker、主题、周期）
│   ├── app_tasks.c/.h       四任务、互斥量、回执队列
│   └── app_hooks.c          FreeRTOS 双钩子（无 .h，库按名字链接）
├── BSP/                     板级驱动
│   ├── bsp_uart.c/.h        USART1 日志、USART2 RS485 收发切换
│   ├── bsp_sensor.c/.h      AHT20：CRC-8、定点换算
│   ├── bsp_wifi.c/.h        ESP8266：状态机、硬复位、环形缓冲
│   └── bsp_led.c/.h         心跳灯与网络灯
├── Protocols/               协议层
│   ├── modbus_rtu_master.c/.h  Modbus RTU 主站
│   └── mqtt_client.c/.h        coreMQTT 封装、四主题、JSON 状态机
├── ThirdParty/              FreeRTOS-Kernel V11.3.0 + coreMQTT v5.0.2
├── Core/                    CubeMX 生成区（仅 USER CODE 块手写）
├── Drivers/                 ST HAL 与 CMSIS
├── MDK-ARM/                 Keil 工程（project.uvprojx）
└── project.ioc              CubeMX 配置
```

---

## 8. 已知限制与调试记录

### 8.1 已知限制

1. **`SEG POLL -> FAIL` 间歇失败**：历史 20%~31%，实测约 8%~12%（52 次成功 / 7 次失败量级）。根因是显示任务优先级（3）低于 MQTT 任务（4），Modbus 收字节是阻塞轮询、每轮等 10 ms，100 ms 窗口被抢占切碎导致首字节超时。单次失败下一轮自动补上，功能可接受；彻底修复需把 USART2 改成中断 + 环形缓冲（可复用 USART3 写法）。
2. **USART2 未开中断**：与上一条同源，属当时的功能冻结取舍。
3. **仅 2.4 GHz**：ESP8266 不支持 5 GHz 频段。
4. **负温显示**：按 `t/10, t%10` 拆位对负数会错（`-12.5` 拆成 `-12.-5`），发送遥测时已用"先取绝对值、符号单独拼前缀"规避；数显表侧未做负温显示。

### 8.2 调试记录

**① `port.c` 的 `uxCriticalNesting` 初值**

移植时若沿用 `0xaaaaaaaa` 毒值，临界区计数永不归零 → `BASEPRI` 恒为 `0x50` → TIM4（优先级 15）被屏蔽 → `HAL_GetTick` 停走 → 卡死在第一个 `HAL_Delay`。现象是"程序不跑、但下载和复位都正常"，极难往 FreeRTOS 移植方向想。改为 `0` 后正常。

**② 数显表写入偶发无响应**

原始做法是阻塞轮询等首字节。日志里 `SEG POLL -> FAIL` 与 MQTT 重连、上报高峰时间点重合，才定位到是任务优先级抢占造成的窗口碎片化，不是接线或波特率问题。

**③ FC03 实现了却"没进镜像"**

`map` 文件里查不到 `MB_ReadHoldingRegisters`，一度以为代码被丢弃。实际是有引用但无存活调用者，链接器按 `Removing` 记录 224 B 后剔除——"map 里找不到"不等于"根本没编译"。

---

## 9. 第三方组件

`ThirdParty/` 内含 FreeRTOS-Kernel V11.3.0 与 coreMQTT v5.0.2，版权归各自作者，遵循其随附许可（MIT）。`Drivers/` 与 `Core/` 中 CubeMX 生成部分版权归 STMicroelectronics。

`ThirdParty/coreMQTT/include/core_mqtt_config.h` 为本工程手写配置（日志重定向到 `bsp_log`、`PACKET_RX_TIMEOUT_MS` 改为 10000）；`FreeRTOS/portable/RVDS/ARM_CM4F/port.c` 有一处 `uxCriticalNesting` 初值修正，详见 8.2 ①。
