## ✅ 一、硬件连接建议（以STM32H7系列为例）

使用**差分输入**（即 IN+ 和 IN−），每个 ADC 模块采样一路电流信号：

| 相位    | ADC模块 | 正输入引脚    | 负输入引脚    | 备注   |
| ------- | ------- | ------------- | ------------- | ------ |
| A相电流 | ADC1    | INP1 (如 PC0) | INN1 (如 PC1) | 差分对 |
| B相电流 | ADC2    | INP1 (如 PC2) | INN1 (如 PC3) | 差分对 |
| C相电流 | ADC3    | INP1 (如 PF3) | INN1 (如 PF4) | 差分对 |

🔧 **注**：

* 每组电流信号通过外部电流传感器（如霍尔或分流电阻+运放差分放大）处理后送入 ADC 的差分输入。
* 必须确保 INP 和 INN 引脚**配对支持差分**模式（参考芯片 datasheet 和参考手册的 ADC 输入表）。

---

## ✅ 二、CubeMX 配置步骤

### 1️⃣ 使能 ADC1、ADC2、ADC3

进入 **Peripherals > ADC1 / ADC2 / ADC3**：

* 模式：`IN1 Differential` （如你截图中 ADC1 设置）
* 其它通道设置为 `Disable`

### 2️⃣ 配置每个 ADC 参数（对三个 ADC 相同设置）：

在 `Parameter Settings` 中：

* `Resolution`：**16-bit**（或按需求）
* `Scan Conversion Mode`：**Disabled**
* `Continuous Mode`：**Disabled**
* `Discontinuous Mode`：**Disabled**
* `End Of Conversion Selection`：**End of single conversion**
* `Overrun behavior`：**Preserved**

在 `ADC_Regular_ConversionMode` 下：

* `Enable Regular Conversions`：**Enable**
* `Number of Conversion`：**1**
* `External Trigger Conversion Source`：

  * 所有 ADC 统一设为某个**定时器的触发输出**（如 `TIM1_TRGO` 或 `TIM8_TRGO2`）
* `External Trigger Edge`：**Rising edge**

这样三个 ADC 将被**同一个定时器同步触发采样**。

### 3️⃣ DMA 配置（可选）

如需将采样结果高速搬运到内存：

* 为每个 ADC 配置独立 DMA 通道（建议使用 `circular` 模式）
* Enable `DMA continuous requests`

---

## ✅ 三、同步采样关键配置（重点）

* **触发源统一**：三个 ADC 的 `External Trigger Conversion Source` 选择**同一个定时器的触发输出**（如 TIM1 或 TIM8）。
* **同时启动**：通过软件配置三组 ADC 形成 `Triple simultaneous mode`（如果你的 STM32 系列支持 ADC multi-mode）。

### STM32H7 示例（使用 HAL 手动配置同步）：

```c
ADC_MultiModeTypeDef multimode = {0};
multimode.Mode = ADC_DUALMODE_REGSIMULT;
HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);
```

---

## ✅ 四、参考连线原理图（示意）

```
    相A电流 ——> 电流传感器 ——> INP1 (PC0)
                                      INN1 (PC1)

    相B电流 ——> 电流传感器 ——> INP1 (PC2)
                                      INN1 (PC3)

    相C电流 ——> 电流传感器 ——> INP1 (PF3)
                                      INN1 (PF4)
```

---

## ✅ 五、调试建议

* 使用 **逻辑分析仪/示波器** 查看定时器触发波形是否同步。
* 开启 ADC 的 **EOC (End Of Conversion) 中断** 或 **DMA 完成中断**，验证三相数据采样是否同步。
* 配合 FreeRTOS 也建议将 ADC 采样放入中断/DMA回调中，提高实时性。

