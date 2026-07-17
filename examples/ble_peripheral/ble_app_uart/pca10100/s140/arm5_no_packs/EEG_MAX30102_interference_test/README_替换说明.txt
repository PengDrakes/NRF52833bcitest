EEG + MAX30102 串扰实验完整代码
================================

一、文件替换
------------
1. 用本包 spo2.c 替换工程中原来的 spo2.c。
2. 用本包 ads1292.c 替换工程中原来的 ads1292.c。
3. 用本包 attention.c 替换工程中实际参与编译的 attention.c。
4. 将 interference_test.h 放入以上 C 文件所在目录，或放入 Keil 已配置的头文件搜索路径。
5. 不需要将 interference_test.h 单独添加到 Keil Source Group；只要编译器能找到即可。
6. 全量重新编译工程，不要只编译单个文件。

注意：用户上传的两个 attention.c 内容相同。工程中只保留/修改实际参与编译的那一个，避免重复定义。

二、自动实验流程
----------------
上电初始化完成后，MAX30102 自动执行：

phase=0  mode=0  OFF             60 秒
phase=1  mode=1  DIGITAL_ONLY    60 秒
phase=2  mode=4  BOTH            60 秒
phase=3  mode=1  DIGITAL_ONLY    60 秒
phase=4  mode=0  OFF             60 秒

总时长约 5 分钟。默认完成一次后停留在最终 OFF 状态。

如需反复循环，在 spo2.c 中修改：

#define MAX30102_TEST_REPEAT 0U

改为：

#define MAX30102_TEST_REPEAT 1U

但循环时相邻两轮之间会形成较长 OFF 洗脱期，适合重复实验。

三、模式定义
------------
mode=0：OFF
MAX30102 进入 shutdown，红光和红外光均关闭，不进行传感器总线读取。

mode=1：DIGITAL_ONLY
MAX30102 保持 SpO2 转换工作，但红光和红外光电流均为 0。用于观察数字电路、I2C、任务调度等影响。

mode=2：RED_ONLY
代码已支持，但自动流程未使用。只开启红光。

mode=3：IR_ONLY
代码已支持，但自动流程未使用。只开启红外光。

mode=4：BOTH
红光和红外光均设为 0x3F，正常执行血氧采集。

四、串口关键记录
----------------
1. 模式切换标记：

XMARK cycle=1 phase=2 mode=4 name=BOTH tick=... red=0x3F ir=0x3F settle_s=5

2. 每秒一条原始 EEG 指标：

EEGX cycle=1 phase=2 mode=4 settle=0 tick=... n=250 mean_x100=... rms_x100=... p2p_x100=... a50_x100=... a100_x100=... ovr=... drop=...

字段说明：
- mean_x100：原始 CH2 均值乘 100，单位为 0.01 uV。
- rms_x100：原始 CH2 去均值 RMS 乘 100，单位为 0.01 uV。
- p2p_x100：原始 CH2 峰峰值乘 100，单位为 0.01 uV。
- a50_x100：原始 CH2 中 50 Hz 正弦峰值幅度乘 100。
- a100_x100：原始 CH2 中 100 Hz 正弦峰值幅度乘 100。
- settle=1：模式切换后的前 5 秒，应从正式统计中删除。
- ovr：ADS1292 累计 DRDY overrun。正常情况下应保持不增长。
- drop：EEGX 日志队列丢弃记录数。正常情况下应保持为 0。

示例：

rms_x100=126 代表 RMS=1.26 uV。
a50_x100=18 代表 50 Hz 幅度=0.18 uV。

五、为什么单独增加 EEGX
----------------------
attention.c 的 ATTQ 和 ATTSP 位于以下滤波之后：

1 Hz 高通 -> 50 Hz 陷波 -> 40 Hz 低通

因此 ATTQ/ATTSP 无法完整证明原始 EEG 中是否出现由 MAX30102 引起的 50 Hz 或 100 Hz 串扰。EEGX 在 attention_push_sample() 之前计算，使用的是 ADS1292R CH2 转换后的原始微伏数据。

六、实验使用要求
----------------
1. 串口使用 115200 波特率。
2. 保存完整串口日志，不要只截图。
3. 被试静坐，电极、导线和 MAX30102 位置保持不变。
4. 每个阶段的 settle=1 数据不要参与统计。
5. 重点比较 phase=0/4、phase=1/3、phase=2：
   - DIGITAL_ONLY - OFF：数字/I2C/任务干扰。
   - BOTH - DIGITAL_ONLY：LED 脉冲与电源纹波影响。
   - BOTH - OFF：MAX30102 总体影响。
6. 如果 ovr 持续增长，应先处理实时性问题，不能直接把 RMS/P2P 上升归因于硬件串扰。

七、资源占用
------------
新增一个低优先级 eegx_logger 任务：
- 栈：256 words（nRF52833 上通常为 1024 bytes）
- 队列：4 条统计记录

EEGX 不在 ADS1292 采样任务中直接 printf，避免 UART 阻塞导致人为丢样。
