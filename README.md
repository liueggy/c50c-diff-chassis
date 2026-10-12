# C50C 两轮差速底盘固件

基于轮趣科技 `WHEELTEC_C50X_2026.09.15` 源码，将 **C50C V4.3控制板**适配到现有四轮两驱差速底盘：两个主动轮、两个从动全向轮，沿用双通道电机驱动板和C50C原配六针OLED。

仓库主要负责底盘运动控制、编码器反馈、IMU采集及串口/CAN通信。当前处于初步适配阶段，后续将继续补充实测参数和硬件验证。

## 硬件

![C50C控制板正反面与接口说明](docs/images/c50c-v4.3.png)

图源：原厂资料中的 `C50C_V4.3_接口详解.png`，保留原图。实际接线以V4.3原理图和实物丝印为准。

| 项目 | 当前硬件与资料信息 |
|---|---|
| 主控制板 | WHEELTEC C50C V4.3 |
| 主控芯片 | STM32F407VET6，Cortex-M4；512 KiB Flash，128 KiB SRAM＋64 KiB CCM RAM |
| 板载IMU | ICM20948；当前源码采集加速度和角速度，未开启磁力计采集 |
| 显示 | C50C原配六针OLED |
| 电机驱动 | 沿用现有双通道驱动板，通过PWM和方向信号控制两台电机 |
| 主动轮 | 旧底盘安装手册标称8寸充气轮，名义直径203.2 mm；有效滚动周长待测 |
| 控制板输入电压 | 官方板图标注10～28 V；现有固件普通模式低于20 V会失能，二者含义不同 |

旧资料附有 **WSDC2412D** 驱动器手册，标称功率输入6.5～28 V、每通道持续12 A（手册测试条件下）、控制侧兼容3.3/5 V。该型号尚需与实物核对，不将资料额定值作为本底盘实测能力。

编码器供电需单独核对：旧底板接口为5 V，C50C编码器接口为3.3 V，不能默认旧线束可以直接套接。信号与接线表见[底盘适配说明](docs/Eggy底盘适配说明.md)。

## 当前状态

- `main`：初始化时保存的源码基线。
- `Eggy`：现有底盘的适配分支，使用Keil编译目标 **`Diff_Car`**。
- 已加入独立底盘配置、左右电机/编码器方向配置，保留PWM驱动和OLED；此配置不使用D50A的RS485轮询。
- 已完成一次Keil编译链接（0错误、0警告）和5组主机测试，尚未完成烧录与实机运行验证。

| 底盘参数 | 当前采用值 | 状态 |
|---|---:|---|
| 减速比 | 28∶1 | 项目当前约定，后续可校核 |
| 编码器线数 | 500 | 项目当前约定，后续可校核 |
| 每轮四倍频原始计数 | 56000 | 按500×28×4计算，新程序不除以8 |
| 左右主动轮中心距 | 未填写 | 待实测 |
| 速度PI | KP=300、KI=300 | 沿用C50C默认值，待实机调参 |

**当前默认禁止电机输出。**轮距未填写且 `EGGY_CONFIG_CONFIRMED=0`，OLED显示 `CFG REQUIRED`；自动前进自检也已关闭。应先测量轮距、核对电气接口与左右方向，再完成配置并进行低速调试。当前车体角速度换算含占位轮距，不应用于里程计。

## 快速加入开发

1. 切换到 `Eggy`，检查已有改动；阅读[开发约定](AGENTS.md)和[底盘适配说明](docs/Eggy底盘适配说明.md)。
2. 用Keil µVision打开 [USER/WHEELTEC.uvprojx](USER/WHEELTEC.uvprojx)，选择 `Diff_Car`。当前工程使用 **ARM Compiler 5.06 update 5**，其他编译器版本需另行验证。
3. 优先查看 [eggy_chassis_config.h](CarType/Inc/eggy_chassis_config.h)，确认参数后再修改控制逻辑。
4. 修改前查询调用关系和影响范围；修改后运行相关测试、构建并检查日志，再提交代码。

```powershell
# 仓库根目录；需本机安装CodeGraph、Python和GCC
codegraph init --yes .   # 本地首次建立索引
codegraph sync .         # 后续修改或切换分支后刷新
codegraph node Robot_Select --file CarType/diff_robot_init.c
codegraph impact Set_Pwm --file BALANCE/balance_task.c --depth 2
python tests/test_eggy_profile.py
```

详细命令与解析限制见[CodeGraph工作流](docs/CodeGraph工作流.md)。静态索引包含不同车型和条件编译分支，不能替代实际目标的编译与测试；动态读取C源码的主机测试需明确执行。

## 源码入口

| 位置 | 用途 |
|---|---|
| `USER/main.c`、`USER/system.c` | 启动、硬件初始化、FreeRTOS任务创建 |
| `CarType/` | 车型与机械参数；本项目从差速配置入手 |
| `BALANCE/balance_task.c` | 100 Hz底盘控制、速度PI、反馈与停机逻辑 |
| `BALANCE/data_task.c`、`uartx_callback.c`、`can_callback.c` | 状态反馈与上位机指令 |
| `HARDWARE/`、`SYSTEM/` | 电机、编码器、OLED、IMU及基础驱动 |
| `FreeRTOS/`、`FWLIB/`、`MiddleWares/`、`USB_HOST/` | 内核、芯片库与中间件 |
| `tests/`、`docs/` | 主机测试、适配说明与协作流程 |

当前是 **C语言＋STM32标准外设库＋FreeRTOS的Keil工程**，未迁移到CMake。仓库保留其他车型和原厂旧文件，以Keil选入的文件为准。编译产物及本地CodeGraph索引不进入版本控制。

基础资料可从仓库中的[源码工程使用指南](源码工程使用指南.pdf)开始阅读；底盘手册、驱动器手册和C50C原理图的来源位置记录在[适配说明](docs/Eggy底盘适配说明.md)。
