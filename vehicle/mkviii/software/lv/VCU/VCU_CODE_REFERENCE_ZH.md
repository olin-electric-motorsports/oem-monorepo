# VCU `vcu.h` / `vcu.c` 代码详解（HAL 初学者版）

本文档覆盖以下两个文件的全部关键元素：
- `vehicle/mkvii/software/lv/vcu/vcu.h`
- `vehicle/mkvii/software/lv/vcu/vcu.c`

目标：逐项说明每个变量（类型、用途）、每个结构体（字段类型与用途）、每个函数（功能、输入、输出、返回值、副作用）。

---

## 1. 文件级常量与宏

### 1.1 `vcu.h` 宏

| 名称 | 定义 | 类型/语义 | 用途 |
|---|---:|---|---|
| `VCU_PERMILLE_MAX` | `1000u` | 无符号整型常量 | 把模拟量统一到千分比（0~1000）范围。 |

### 1.2 `vcu.c` 默认标定宏

| 名称 | 值 | 用途 |
|---|---:|---|
| `VCU_DEFAULT_ADC_MAX_COUNTS` | `4095` | 12-bit ADC 默认最大计数值。 |
| `VCU_DEFAULT_APPS_MISMATCH_THRESHOLD_PERMILLE` | `100` | APPS1/APPS2 允许差值阈值（千分比）。 |
| `VCU_DEFAULT_APPS_IMPLAUS_PERSIST_MS` | `100` | APPS 异常持续多少 ms 才锁存。 |
| `VCU_DEFAULT_BRAKE_SENSOR_IMPLAUS_PERSIST_MS` | `100` | 刹车传感器异常持续多少 ms 判故障。 |
| `VCU_DEFAULT_BRAKE_THROTTLE_TRIGGER_PERMILLE` | `250` | 刹车与油门冲突触发阈值。 |
| `VCU_DEFAULT_BRAKE_THROTTLE_CLEAR_PERMILLE` | `50` | 冲突清除阈值。 |
| `VCU_DEFAULT_HARD_BRAKE_THRESHOLD_PERMILLE` | `300` | “重刹”阈值。 |
| `VCU_DEFAULT_BSPD_TRIGGER_DELAY_MS` | `500` | BSPD 条件持续时间阈值。 |
| `VCU_DEFAULT_TORQUE_COMMAND_MAX` | `2540` | 最大扭矩命令。 |
| `VCU_DEFAULT_THROTTLE_DEADBAND_PERMILLE` | `20` | 油门死区。 |
| `VCU_DEFAULT_THROTTLE_ZERO_CLEAR_PERMILLE` | `50` | APPS 锁存故障清除条件阈值。 |
| `VCU_DEFAULT_FILTER_SHIFT` | `2` | IIR 滤波强度（位移）。 |
| `VCU_DEFAULT_HEARTBEAT_TOGGLE_MS` | `500` | 心跳翻转周期。 |
| `VCU_DEFAULT_INVERTER_COMMAND_PERIOD_MS` | `10` | 逆变器命令发送周期。 |

### 1.3 `vcu.c` 故障掩码宏

| 名称 | 类型 | 用途 |
|---|---|---|
| `VCU_BLOCKING_FAULT_MASK` | 位掩码表达式 | 指定“阻断扭矩输出”的故障集合，用于 `s_state.blocking_fault_bits`。 |

---

## 2. 枚举类型

### 2.1 `vcu_mode_e`

| 枚举值 | 含义 |
|---|---|
| `VCU_MODE_INIT` | 初始化状态。 |
| `VCU_MODE_NOT_READY` | 未 ready-to-drive。 |
| `VCU_MODE_FAULT` | 存在阻断性故障。 |
| `VCU_MODE_RUN` | 正常运行可出扭矩。 |

### 2.2 `vcu_fault_bit_e`

位定义（`uint32_t` 位域）：

| 位 | 枚举值 | 含义 |
|---:|---|---|
| bit0 | `VCU_FAULT_APPS1_OUT_OF_RANGE` | APPS1 超标定范围。 |
| bit1 | `VCU_FAULT_APPS2_OUT_OF_RANGE` | APPS2 超标定范围。 |
| bit2 | `VCU_FAULT_APPS_MISMATCH` | APPS1/2 不一致。 |
| bit3 | `VCU_FAULT_APPS_TIMEOUT_LATCHED` | APPS 异常持续并锁存。 |
| bit4 | `VCU_FAULT_BRAKE_SENSOR_TIMEOUT` | 刹车传感器异常持续超时。 |
| bit5 | `VCU_FAULT_BRAKE_THROTTLE_IMPLAUS` | 刹车与油门冲突锁存。 |
| bit6 | `VCU_FAULT_BSPD_POWER_LATCHED` | BSPD 触发并锁存。 |
| bit7 | `VCU_FAULT_SHUTDOWN_BSPD_OPEN` | BSPD 回路开路。 |
| bit8 | `VCU_FAULT_INERTIA_SWITCH_OPEN` | 惯性开关开路。 |
| bit9 | `VCU_FAULT_INVERTER_REPORTED` | 逆变器上报故障。 |
| bit10 | `VCU_FAULT_INTERNAL_ADC` | ADC 读取内部错误。 |

---

## 3. 结构体与字段详解（全部字段）

## 3.1 `vcu_can_inputs_s`

| 字段 | 类型 | 用途 |
|---|---|---|
| `ready_to_drive` | `bool` | 来自 CAN 的 ready-to-drive 许可。 |
| `inverter_fault_active` | `bool` | 逆变器故障状态反馈。 |
| `inverter_enable_feedback` | `bool` | 逆变器使能反馈。 |

## 3.2 `vcu_adc_samples_s`

| 字段 | 类型 | 用途 |
|---|---|---|
| `apps1_raw` | `uint16_t` | APPS1 原始 ADC 计数。 |
| `apps2_raw` | `uint16_t` | APPS2 原始 ADC 计数。 |
| `brake_pressure_raw` | `uint16_t` | 刹车压力原始计数。 |
| `brake_pressure_filtered_raw` | `uint16_t` | 刹车压力滤波后计数（若外部已滤波）。 |
| `motor_current_sense` | `bool` | 电机电流门限数字信号（例如超过 5kW）。 |
| `apps1_valid` | `bool` | `apps1_raw` 是否有效。 |
| `apps2_valid` | `bool` | `apps2_raw` 是否有效。 |
| `brake_pressure_valid` | `bool` | `brake_pressure_raw` 是否有效。 |
| `brake_pressure_filtered_valid` | `bool` | `brake_pressure_filtered_raw` 是否有效。 |
| `motor_current_sense_valid` | `bool` | `motor_current_sense` 是否有效。 |

## 3.3 `vcu_hw_s`（HAL 句柄与 GPIO 映射）

| 字段 | 类型 | 用途 |
|---|---|---|
| `hadc_apps1` | `ADC_HandleTypeDef*` | APPS1 ADC 句柄（必需）。 |
| `hadc_apps2` | `ADC_HandleTypeDef*` | APPS2 ADC 句柄（必需）。 |
| `hadc_brake_pressure` | `ADC_HandleTypeDef*` | 刹车压力 ADC 句柄（可选）。 |
| `hadc_brake_pressure_filtered` | `ADC_HandleTypeDef*` | 滤波刹车压力 ADC 句柄（可选）。 |
| `motor_current_sense_port` | `GPIO_TypeDef*` | 电机电流数字输入端口。 |
| `motor_current_sense_pin` | `uint16_t` | 电机电流数字输入引脚。 |
| `motor_current_sense_active_state` | `GPIO_PinState` | 电机电流数字信号激活电平。 |
| `hiwdg` | `IWDG_HandleTypeDef*` | 独立看门狗句柄（可选）。 |
| `brake_gate_port` | `GPIO_TypeDef*` | 刹车门限数字输入端口。 |
| `brake_gate_pin` | `uint16_t` | 刹车门限数字输入引脚。 |
| `brake_gate_active_state` | `GPIO_PinState` | 刹车门限“激活”电平。 |
| `bspd_5kw_port` | `GPIO_TypeDef*` | BSPD 5kW 输入端口。 |
| `bspd_5kw_pin` | `uint16_t` | BSPD 5kW 输入引脚。 |
| `bspd_5kw_active_state` | `GPIO_PinState` | BSPD 5kW 激活电平。 |
| `ss_bspd_port` | `GPIO_TypeDef*` | Shutdown-BSPD 回路输入端口。 |
| `ss_bspd_pin` | `uint16_t` | Shutdown-BSPD 输入引脚。 |
| `ss_bspd_closed_state` | `GPIO_PinState` | 回路“闭合”对应电平。 |
| `ss_inertia_port` | `GPIO_TypeDef*` | 惯性开关回路输入端口。 |
| `ss_inertia_pin` | `uint16_t` | 惯性开关输入引脚。 |
| `ss_inertia_closed_state` | `GPIO_PinState` | 惯性回路“闭合”电平。 |
| `inverter_enable_port` | `GPIO_TypeDef*` | 逆变器使能输出端口。 |
| `inverter_enable_pin` | `uint16_t` | 逆变器使能输出引脚。 |
| `inverter_enable_active_state` | `GPIO_PinState` | 逆变器使能激活电平。 |
| `fault_led_port` | `GPIO_TypeDef*` | 故障灯输出端口。 |
| `fault_led_pin` | `uint16_t` | 故障灯引脚。 |
| `indicator_led_active_state` | `GPIO_PinState` | 指示灯激活电平。 |
| `heartbeat_led_port` | `GPIO_TypeDef*` | 心跳灯输出端口。 |
| `heartbeat_led_pin` | `uint16_t` | 心跳灯引脚。 |
| `heartbeat_led_active_state` | `GPIO_PinState` | 心跳灯激活电平。 |

## 3.4 `vcu_calib_s`（标定）

| 字段 | 类型 | 用途 |
|---|---|---|
| `apps1_min_counts` | `uint16_t` | APPS1 最小有效 ADC。 |
| `apps1_max_counts` | `uint16_t` | APPS1 最大有效 ADC。 |
| `apps1_inverted` | `bool` | APPS1 是否反向。 |
| `apps2_min_counts` | `uint16_t` | APPS2 最小有效 ADC。 |
| `apps2_max_counts` | `uint16_t` | APPS2 最大有效 ADC。 |
| `apps2_inverted` | `bool` | APPS2 是否反向。 |
| `brake_pressure_min_counts` | `uint16_t` | 刹车压力最小 ADC。 |
| `brake_pressure_max_counts` | `uint16_t` | 刹车压力最大 ADC。 |
| `apps_mismatch_threshold_permille` | `uint16_t` | APPS 差值容差阈值。 |
| `apps_implausibility_persist_ms` | `uint16_t` | APPS 异常持续阈值。 |
| `brake_sensor_implausibility_persist_ms` | `uint16_t` | 刹车传感器异常持续阈值。 |
| `brake_throttle_trigger_permille` | `uint16_t` | 刹车+油门冲突触发阈值。 |
| `brake_throttle_clear_permille` | `uint16_t` | 冲突清除阈值。 |
| `hard_brake_threshold_permille` | `uint16_t` | 重刹阈值。 |
| `bspd_trigger_delay_ms` | `uint16_t` | BSPD 锁存延时阈值。 |
| `torque_command_max` | `int16_t` | 最大扭矩命令。 |
| `throttle_deadband_permille` | `uint16_t` | 油门死区阈值。 |
| `throttle_zero_clear_permille` | `uint16_t` | APPS 锁存清除阈值。 |
| `filter_shift` | `uint8_t` | IIR 滤波位移（0=不滤波）。 |
| `heartbeat_toggle_ms` | `uint16_t` | 心跳灯翻转周期。 |
| `inverter_command_period_ms` | `uint16_t` | 逆变器命令发布周期。 |
| `disable_inverter_on_fault` | `bool` | 故障时是否拉低逆变器使能。 |

## 3.5 `vcu_state_s`（运行状态）

| 字段 | 类型 | 用途 |
|---|---|---|
| `apps1_raw` | `uint16_t` | APPS1 原始值。 |
| `apps2_raw` | `uint16_t` | APPS2 原始值。 |
| `brake_pressure_raw` | `uint16_t` | 刹车压力原始值。 |
| `brake_pressure_filtered_raw` | `uint16_t` | 刹车压力滤波前端值。 |
| `motor_current_sense` | `bool` | 电机电流门限数字信号当前状态。 |
| `apps1_permille` | `uint16_t` | APPS1 千分比。 |
| `apps2_permille` | `uint16_t` | APPS2 千分比。 |
| `apps_safe_permille` | `uint16_t` | 两踏板取较小值（安全输出）。 |
| `apps_high_permille` | `uint16_t` | 两踏板取较大值（冲突判断）。 |
| `brake_pressure_permille` | `uint16_t` | 刹车压力千分比。 |
| `brake_gate` | `bool` | 刹车门限数字输入。 |
| `bspd_5kw` | `bool` | BSPD 5kW 数字输入。 |
| `ss_bspd_closed` | `bool` | BSPD 回路闭合状态。 |
| `ss_inertia_closed` | `bool` | 惯性回路闭合状态。 |
| `apps1_out_of_range` | `bool` | APPS1 是否越界。 |
| `apps2_out_of_range` | `bool` | APPS2 是否越界。 |
| `apps_mismatch` | `bool` | APPS 是否不一致。 |
| `brake_sensor_out_of_range` | `bool` | 刹车压力是否越界。 |
| `apps_implaus_latched` | `bool` | APPS 故障锁存位。 |
| `brake_throttle_implaus_latched` | `bool` | 刹车油门冲突锁存位。 |
| `bspd_latched` | `bool` | BSPD 锁存位。 |
| `hard_brake_detected` | `bool` | 是否检测到重刹。 |
| `over_5kw_detected` | `bool` | 是否检测到过 5kW 条件。 |
| `adc_read_error` | `bool` | ADC 读取错误。 |
| `ready_to_drive` | `bool` | CAN ready-to-drive。 |
| `inverter_fault_active` | `bool` | CAN 逆变器故障。 |
| `inverter_enable_feedback` | `bool` | CAN 逆变器使能反馈。 |
| `inverter_enable_command` | `bool` | 输出给逆变器使能命令。 |
| `torque_command` | `int16_t` | 输出给逆变器的扭矩命令。 |
| `heartbeat` | `bool` | 心跳输出状态。 |
| `heartbeat_elapsed_ms` | `uint16_t` | 心跳累计计时。 |
| `apps_implaus_timer_ms` | `uint16_t` | APPS 异常累计计时。 |
| `brake_sensor_implaus_timer_ms` | `uint16_t` | 刹车传感器异常累计计时。 |
| `bspd_timer_ms` | `uint16_t` | BSPD 条件累计计时。 |
| `fault_bits` | `uint32_t` | 全部故障位。 |
| `blocking_fault_bits` | `uint32_t` | 阻断型故障位。 |
| `mode` | `vcu_mode_e` | VCU 当前模式。 |
| `inverter_command_publish_elapsed_ms` | `uint16_t` | 距离上次逆变器命令发布经过时间。 |

## 3.6 回调函数类型

| 类型 | 原型 | 用途 |
|---|---|---|
| `vcu_watchdog_refresh_f` | `void (*)(void* user_ctx)` | 用户自定义看门狗刷新。 |
| `vcu_bootloader_service_f` | `void (*)(void* user_ctx)` | 用户自定义 bootloader 后台服务。 |
| `vcu_read_can_inputs_f` | `void (*)(void* user_ctx, vcu_can_inputs_s* can_inputs)` | 用户读取/更新 CAN 输入。 |
| `vcu_read_adc_samples_f` | `bool (*)(void* user_ctx, vcu_adc_samples_s* samples)` | 用户提供 ADC 采样（DMA/ISR 路径）。 |
| `vcu_publish_inverter_command_f` | `void (*)(void* user_ctx, int16_t torque_command, bool inverter_enable)` | 发布逆变器命令。 |
| `vcu_publish_state_f` | `void (*)(void* user_ctx, const vcu_state_s* state)` | 发布状态调试消息。 |

## 3.7 `vcu_hooks_s`

| 字段 | 类型 | 用途 |
|---|---|---|
| `user_ctx` | `void*` | 用户上下文指针，回调透传。 |
| `watchdog_refresh` | `vcu_watchdog_refresh_f` | 回调：看门狗服务。 |
| `bootloader_service` | `vcu_bootloader_service_f` | 回调：bootloader 服务。 |
| `read_can_inputs` | `vcu_read_can_inputs_f` | 回调：读 CAN 输入。 |
| `read_adc_samples` | `vcu_read_adc_samples_f` | 回调：提供 ADC 样本。 |
| `publish_inverter_command` | `vcu_publish_inverter_command_f` | 回调：发扭矩/使能命令。 |
| `publish_vcu_status` | `vcu_publish_state_f` | 回调：发 VCU 状态。 |
| `publish_throttle_debug` | `vcu_publish_state_f` | 回调：发油门调试。 |
| `publish_bspd_debug` | `vcu_publish_state_f` | 回调：发 BSPD 调试。 |

---

## 4. `vcu.c` 文件级静态变量（全部）

| 变量 | 类型 | 初值 | 用途 |
|---|---|---|---|
| `s_hw` | `vcu_hw_s` | 全 0 | 保存硬件映射。 |
| `s_calib` | `vcu_calib_s` | 全 0 | 保存当前标定。 |
| `s_hooks` | `vcu_hooks_s` | 全 0 | 保存回调函数集合。 |
| `s_can_inputs` | `vcu_can_inputs_s` | 全 0 | 最近的 CAN 输入快照。 |
| `s_state` | `vcu_state_s` | 全 0 | VCU 全部运行状态。 |
| `s_hw_initialized` | `bool` | `false` | `vcu_init` 是否成功执行。 |
| `s_tick_1ms` | `volatile bool` | `false` | 1ms 任务触发标志。 |
| `s_tick_10ms` | `volatile bool` | `false` | 10ms 任务触发标志。 |

---

## 5. 函数详解（全部函数，含 static）

说明格式：
- 功能：做什么。
- 输入：参数类型和含义。
- 输出：返回值/输出参数。
- 局部变量：函数内定义变量及用途。
- HAL 相关：直接使用的 HAL API。
- 副作用：会改动的全局状态。

## 5.1 内联辅助函数

### `static inline bool vcu_gpio_is_configured(GPIO_TypeDef* port, uint16_t pin)`
- 功能：判断 GPIO 配置是否有效（`port != NULL && pin != 0`）。
- 输入：`port`、`pin`。
- 输出：`bool`。
- 局部变量：无。
- HAL 相关：无。
- 副作用：无。

### `static inline uint16_t vcu_min_u16(uint16_t a, uint16_t b)`
- 功能：返回较小值。
- 输入：`a`、`b`。
- 输出：`uint16_t`。
- 局部变量：无。
- HAL 相关：无。
- 副作用：无。

### `static inline uint16_t vcu_abs_diff_u16(uint16_t a, uint16_t b)`
- 功能：无符号绝对差。
- 输入：`a`、`b`。
- 输出：`uint16_t`。
- 局部变量：无。
- HAL 相关：无。
- 副作用：无。

### `static inline bool vcu_bool_from_pin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState active_state, bool default_if_unconfigured)`
- 功能：读 GPIO 并映射成逻辑量；未配置时返回默认值。
- 输入：GPIO 端口/引脚、激活电平、未配置默认值。
- 输出：`bool` 逻辑状态。
- 局部变量：无。
- HAL 相关：`HAL_GPIO_ReadPin`。
- 副作用：无。

### `static inline void vcu_write_pin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState active_state, bool active)`
- 功能：按“激活态定义”写 GPIO，自动处理高有效/低有效。
- 输入：GPIO 端口/引脚、激活电平、目标逻辑态。
- 输出：无。
- 局部变量：
  - `GPIO_PinState pin_state`：最终写入电平。
- HAL 相关：`HAL_GPIO_WritePin`。
- 副作用：驱动硬件引脚。

### `static inline uint16_t vcu_increment_sat_u16(uint16_t value)`
- 功能：`uint16_t` 饱和加一（到 `UINT16_MAX` 不再增加）。
- 输入：`value`。
- 输出：递增后的值。
- 局部变量：无。
- HAL 相关：无。
- 副作用：无。

### `static inline bool vcu_hw_ready(void)`
- 功能：读取模块是否完成初始化。
- 输入：无。
- 输出：`s_hw_initialized`。
- 局部变量：无。
- HAL 相关：无。
- 副作用：无。

## 5.2 采样与缩放函数

### `static HAL_StatusTypeDef vcu_read_adc_once(ADC_HandleTypeDef* hadc, uint16_t* out_counts)`
- 功能：对单个 ADC 做一次阻塞采样。
- 输入：
  - `hadc`：ADC HAL 句柄。
  - `out_counts`：输出计数地址。
- 输出：
  - 返回 `HAL_OK`/`HAL_ERROR`。
  - 成功时写 `*out_counts`。
- 局部变量：无。
- HAL 相关：
  - `HAL_ADC_Start`
  - `HAL_ADC_PollForConversion`
  - `HAL_ADC_GetValue`
  - `HAL_ADC_Stop`
- 副作用：触发 ADC 外设一次转换。

### `static uint16_t vcu_scale_counts_to_permille(uint16_t raw, uint16_t raw_min, uint16_t raw_max, bool inverted, bool* out_of_range)`
- 功能：ADC 计数映射到 0~1000 千分比，含限幅、反向处理、越界标记。
- 输入：原始值、最小/最大标定、是否反向、越界输出指针。
- 输出：千分比值。
- 局部变量：
  - `uint16_t clamped`：限幅后的值。
  - `uint32_t scaled`：中间缩放结果（防溢出）。
  - `bool local_out_of_range`：局部越界标记。
- HAL 相关：无。
- 副作用：若 `out_of_range != NULL` 则写该标记。

### `static uint16_t vcu_iir_filter_permille(uint16_t previous, uint16_t sample, uint8_t shift)`
- 功能：一阶 IIR 滤波（位移实现）。
- 输入：上一输出、当前采样、滤波位移。
- 输出：滤波后千分比。
- 局部变量：
  - `int32_t filtered`：有符号中间量。
- HAL 相关：无。
- 副作用：无。

### `static void vcu_read_adc_signal(ADC_HandleTypeDef* hadc, uint16_t* target, bool required)`
- 功能：读取一个 ADC 通道，失败时设置 `s_state.adc_read_error`。
- 输入：ADC 句柄、目标地址、是否必需。
- 输出：无（通过 `target` 输出值）。
- 局部变量：
  - `uint16_t sample`：临时样本。
- HAL 相关：间接调用 `vcu_read_adc_once`。
- 副作用：
  - 成功时写 `*target`。
  - 失败时置 `s_state.adc_read_error = true`。

## 5.3 后台与 IO 应用

### `static void vcu_background_service(void)`
- 功能：后台服务，包括硬件看门狗刷新和用户钩子执行。
- 输入：无。
- 输出：无。
- 局部变量：无。
- HAL 相关：`HAL_IWDG_Refresh`。
- 副作用：可能刷新 IWDG，调用用户回调。

### `static void vcu_apply_outputs(void)`
- 功能：把 `s_state` 中的命令映射到 GPIO 输出（逆变器使能、故障灯、心跳灯）。
- 输入：无。
- 输出：无。
- 局部变量：
  - `bool has_fault`：`fault_bits` 是否非零。
- HAL 相关：间接通过 `vcu_write_pin` 调用 `HAL_GPIO_WritePin`。
- 副作用：更新外部 GPIO 电平。

## 5.4 1ms 任务分解函数

### `static void vcu_read_inputs_1ms(void)`
- 功能：采集 1ms 周期输入（ADC + GPIO）。
- 输入：无。
- 输出：无。
- 局部变量：
  - `vcu_adc_samples_s samples`：外部采样回调缓存。
  - `bool used_external_samples`：是否采用外部 ADC 样本。
- HAL 相关：
  - 若未用外部样本，调用 `vcu_read_adc_signal` -> ADC HAL。
  - GPIO 输入通过 `vcu_bool_from_pin` -> `HAL_GPIO_ReadPin`。
- 副作用：更新 `s_state` 原始输入字段、输入逻辑字段、`adc_read_error`。

### `static void vcu_normalize_and_filter_1ms(void)`
- 功能：把原始输入归一化到千分比并滤波，计算衍生量（safe/high/mismatch、重刹、过 5kW）。
- 输入：无。
- 输出：无。
- 局部变量：
  - `uint16_t apps1_norm`
  - `uint16_t apps2_norm`
  - `uint16_t brake_pressure_norm`
- HAL 相关：无。
- 副作用：更新 `s_state` 的归一化值、判定标志和衍生状态。

### `static void vcu_update_fault_manager_1ms(void)`
- 功能：根据当前状态更新故障位和锁存逻辑。
- 输入：无。
- 输出：无。
- 局部变量：
  - `uint32_t fault_bits`：本周期累积故障位。
  - `bool apps_implausible_now`：APPS 当前是否不可置信。
- HAL 相关：无。
- 副作用：
  - 更新 `s_state.fault_bits` / `blocking_fault_bits`。
  - 更新各类 timer 和 latched 位。

### `static void vcu_update_torque_command_1ms(void)`
- 功能：基于 ready/fault/油门计算扭矩命令与逆变器使能。
- 输入：无。
- 输出：无。
- 局部变量：
  - `bool has_fault`：是否有阻断故障。
  - `uint32_t torque`：中间扭矩计算值。
- HAL 相关：无。
- 副作用：更新 `s_state` 的 `torque_command`、`inverter_enable_command`、`mode`。

## 5.5 对外公开函数（`vcu.h` 声明）

### `void vcu_load_default_calibration(vcu_calib_s* calib)`
- 功能：填充默认标定值。
- 输入：`calib` 输出对象指针。
- 输出：无。
- 局部变量：无。
- HAL 相关：无。
- 副作用：写入 `*calib`。

### `HAL_StatusTypeDef vcu_init(const vcu_hw_s* hw, const vcu_calib_s* calib, const vcu_hooks_s* hooks)`
- 功能：初始化模块（参数校验、拷贝配置、清空状态、输出初值）。
- 输入：
  - `hw`：硬件配置（必需）。
  - `calib`：标定参数（必需）。
  - `hooks`：回调（可空）。
- 输出：`HAL_OK` 或 `HAL_ERROR`。
- 局部变量：无。
- HAL 相关：间接调用 `vcu_apply_outputs` -> GPIO HAL。
- 副作用：
  - 赋值 `s_hw/s_calib/s_hooks/s_state/s_can_inputs`。
  - 置 `s_hw_initialized=true`，复位 tick 标志。

### `void vcu_request_1ms_tick(void)`
- 功能：置位 1ms 执行标志。
- 输入/输出：无。
- 局部变量：无。
- HAL 相关：无。
- 副作用：`s_tick_1ms = true`。

### `void vcu_request_10ms_tick(void)`
- 功能：置位 10ms 执行标志。
- 输入/输出：无。
- 局部变量：无。
- HAL 相关：无。
- 副作用：`s_tick_10ms = true`。

### `void vcu_set_can_inputs(const vcu_can_inputs_s* can_inputs)`
- 功能：外部直接写入 CAN 输入快照。
- 输入：`can_inputs` 指针。
- 输出：无。
- 局部变量：无。
- HAL 相关：无。
- 副作用：更新 `s_can_inputs`。

### `void vcu_clear_latched_faults(void)`
- 功能：清除可清除的锁存故障与相关计时器。
- 输入/输出：无。
- 局部变量：无。
- HAL 相关：无。
- 副作用：清零 `apps_implaus_latched`、`brake_throttle_implaus_latched`、`bspd_latched` 及对应计时器。

### `HAL_StatusTypeDef vcu_step_1ms(void)`
- 功能：执行 1ms 主逻辑链路。
- 输入：无。
- 输出：`HAL_OK` 或 `HAL_ERROR`（ADC 错误时返回 `HAL_ERROR`）。
- 局部变量：无。
- HAL 相关：
  - 经子函数使用 ADC/GPIO。
  - 可能调用 `publish_inverter_command` 回调发布命令。
- 副作用：更新 `s_state` 大部分运行字段并刷新输出。

### `HAL_StatusTypeDef vcu_step_10ms(void)`
- 功能：执行 10ms 逻辑（心跳与状态发布）。
- 输入：无。
- 输出：`HAL_OK` 或 `HAL_ERROR`（仅在未初始化时 `HAL_ERROR`）。
- 局部变量：无。
- HAL 相关：通过 `vcu_apply_outputs` 写 GPIO。
- 副作用：更新心跳状态并调用状态发布回调。

### `HAL_StatusTypeDef vcu_main_loop_iteration(void)`
- 功能：主循环单次迭代：后台服务、可选 CAN 读取、按 tick 调度 1ms/10ms。
- 输入：无。
- 输出：`HAL_OK`/`HAL_ERROR`。
- 局部变量：
  - `HAL_StatusTypeDef status`：本次迭代汇总状态。
  - `vcu_can_inputs_s callback_inputs`：回调可读写的 CAN 输入副本。
- HAL 相关：经 `vcu_background_service` 使用 IWDG HAL。
- 副作用：执行任务并消费 tick 标志。

### `const vcu_state_s* vcu_get_state(void)`
- 功能：返回状态结构体只读指针。
- 输入：无。
- 输出：`const vcu_state_s*`。
- 局部变量：无。
- HAL 相关：无。
- 副作用：无。

---

## 6. HAL 初学者重点映射

### 6.1 句柄的角色
- `ADC_HandleTypeDef*` / `IWDG_HandleTypeDef*` / `GPIO_TypeDef*` 都由外部工程（通常 CubeMX 生成）初始化。
- 本模块只保存这些句柄并调用 HAL API，不负责底层外设时钟和 GPIO 复用配置。

### 6.2 本模块直接使用到的 HAL API

| HAL API | 在本模块中的作用 |
|---|---|
| `HAL_ADC_Start` | 启动 ADC 单次转换。 |
| `HAL_ADC_PollForConversion` | 阻塞等待转换完成。 |
| `HAL_ADC_GetValue` | 读取转换结果。 |
| `HAL_ADC_Stop` | 结束转换。 |
| `HAL_GPIO_ReadPin` | 读取数字输入。 |
| `HAL_GPIO_WritePin` | 写数字输出。 |
| `HAL_IWDG_Refresh` | 刷新独立看门狗。 |

### 6.3 扩展方式
- 若你后续把 ADC 改为 DMA/中断采样，不需要改主要业务逻辑，只需实现 `read_adc_samples` 回调并返回 `true`。
- 若你要通过 CAN 发逆变器命令，实现 `publish_inverter_command` 回调即可。

---

## 7. 使用建议（对接时最容易错的点）

1. `vcu_init` 前必须保证 `hadc_apps1`、`hadc_apps2` 已经初始化完成，否则直接 `HAL_ERROR`。
2. GPIO 的 `active_state/closed_state` 必须和硬件电平一致；否则逻辑会全反。
3. `vcu_request_1ms` / `vcu_request_10ms` 建议在定时中断里置位，`vcu_main_loop_iteration` 在主循环持续调用。
4. `vcu_step_1ms` 返回 `HAL_ERROR` 不一定代表系统停机，当前实现主要用于提示 ADC 采样路径有错误。
