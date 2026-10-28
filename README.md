# C50C 两轮差速底盘

基于轮趣科技源码，适配现有四轮两驱底盘。

![C50C V4.3接口图](docs/images/c50c-v4.3.png)

*原厂接口图；接线以V4.3原理图和实物为准。*

## 硬件与参数

| 项目 | 配置 / 状态 |
|---|---|
| 控制板 | C50C V4.3，STM32F407VET6 |
| 内存 | Flash 512 KiB；SRAM 128 KiB＋CCM 64 KiB |
| IMU / 显示 | ICM20948 / 原配六针OLED |
| 电机驱动 | 资料型号WSDC2412D（待核对实物），双通道PWM＋方向信号 |
| 底盘 | 两主动轮＋两从动全向轮 |
| 主动轮直径 | 手册标称8寸（203.2 mm），待标定 |
| 减速比 / 编码器 | 当前按28∶1 / 500线；四倍频56000次/轮圈 |
| 轮距 / PI | 轮距550 mm（用户实测）；KP=300、KI=300，待调参 |
| 板输入电压 | 板图标称10～28 V；当前12 V供电，固件低于9.5 V停机 |

## 驱动板与接线

![WSDC2412D驱动板，摘自原厂数据手册](docs/images/wsdc2412d-board.jpg)

| WSDC2412D手册参数 | 数值 |
|---|---|
| 功率输入 / 持续输出 | 6.5～28 V / 每通道12 A（手册测试条件） |
| 控制接口 | PWM、INA、INB；逻辑输入3～5.5 V |
| 尺寸 | 50×50×12.5 mm |

A/B电机通道与编码器的左右对应和方向已实机验证；下表的CN1编号按驱动手册，排线逐针对应仍以原理图与实物为准。

| C50C信号 | WSDC2412D CN1 |
|---|---|
| 左轮A：PC9 / PB13 / PB12 | 2 PWM1 / 3 INA1 / 4 INB1 |
| 右轮B：PC8 / PC0 / PE15 | 7 PWM2 / 8 INA2 / 9 INB2 |
| 控制侧3.3 V / GND | 1、6 VCC / 5、10 GND（10在图中标为X） |
| 电池正、负极 | 功率端P+、P− |
| 左、右电机 | 功率端M1、M2 |

编码器：左轮接PA15/PB3，右轮接PA6/PA7。OLED沿用C50C原配六针接口。编码器反馈及方向已验证；更换线束或外设时仍需核对供电、电平与插头顺序。

<details>
<summary>驱动器典型接线图与原理框图（手册原页）</summary>

![驱动器典型接线，手册第5页](docs/images/wsdc2412d-wiring.png)

![驱动器原理框图，手册第1页](docs/images/wsdc2412d-overview.png)

接线图标注6.5～29 V，与参数表不一致；使用上限按参数表28 V。

</details>

## 原理图与手册

| 资料 | 图片预览 | 原始PDF |
|---|---|---|
| C50C V4.3原理图 | [查看4页原理图](docs/hardware/README.md) | [PDF](https://raw.githubusercontent.com/liueggy/c50c-diff-chassis/Eggy/docs/hardware/C50C_V4.3_%E5%8E%9F%E7%90%86%E5%9B%BE.pdf) |
| C50C V4.3资源分配 | [查看资源表](docs/hardware/README.md#资源分配) | [PDF](https://raw.githubusercontent.com/liueggy/c50c-diff-chassis/Eggy/docs/hardware/C50C_V4.3_%E8%B5%84%E6%BA%90%E5%88%86%E9%85%8D.pdf) |
| WSDC2412D手册 | [接线图](docs/images/wsdc2412d-wiring.png) / [原理框图](docs/images/wsdc2412d-overview.png) | [PDF](https://raw.githubusercontent.com/liueggy/c50c-diff-chassis/Eggy/docs/hardware/WSDC2412D_%E6%95%B0%E6%8D%AE%E6%89%8B%E5%86%8C_V2.0.pdf) |

驱动器资料提供原理框图，未提供完整元件级原理图。

更多资料集中在 [C50C 开发与外围硬件手册](docs/manuals/README.md)：底盘开发、UART/CAN 协议、STM32F407 数据手册与 RM0090、ICM20948、CH9102F、CAN 收发器、D50A 和 OLED。索引标明适用版本、用途和资料缺口，PDF 可直接下载，克隆后也可按相对路径查阅。

## 当前状态（2026-10-15）

- `main`：源码基线；`Eggy`：底盘适配（当前分支，目标 `Diff_Car`）。
- 已按 **12 V 固定供电**完成适配：`CHASSIS_CONFIG_CONFIRMED=1`，普通 `Diff_Car` 构建允许电机输出；低压门控下调为 9.5 V（`CHASSIS_STOP/LOW/CLEAR_VOLTAGE_V`）。
- 电机方向、编码器符号、轮距（550 mm）已实机核对；**100% 占空比实测约 950 mm/s**。
- OLED 为**两页中文界面**：第 1 页概览（状态/电压/蓝牙功能/串口通信），第 2 页动态（左右轮目标·实测、占空比、角速度）；**长按用户按键（PC5）翻页**。
- USART3与JDY-33蓝牙均可控制并回传状态，首个零帧获得独占控制权；提供释放指令及6字节owner回报，旧蓝牙APP调参协议关闭。
- 待做：JDY-33配对、双通道交接、断连/超时/越界的实车联调记录；里程计与真实速度标定。
- 本仓库交付固件和接口；ROS 2节点由上位机负责人在其仓库实现。
- 已实现实际使能/电压门控、恢复后零帧重解锁及异常当周期清PWM/PI；新增行为仍需实车联调记录。

## ROS 2 串口协议（Diff_Car，v1.2）

Jetson NX 使用用户指定的 **Type-C 串口 3** 与底盘通信；USART3 配置为 **115200 baud、8N1、无硬件流控**。实际接入时确认 Jetson 枚举的设备及收发对应串口 3。JDY-33 使用 UART4，默认 **9600、8N1**；两路独立解析、互斥控制。USART1 用于下载和状态回传，不接收本底盘运动指令。

ROS 端由桥接节点订阅 `/cmd_vel`（`geometry_msgs/msg/Twist`），将 `linear.x`（m/s）和 `angular.z`（rad/s）乘 1000 转成下述二进制帧。`linear.x > 0` 前进，`angular.z > 0` 左转；横向速度须为 0。线速度允许 ±0.7 m/s，角速度允许 ±1.0 rad/s；越界会判错，不能自动视为限幅。本协议不同于原厂 WHEELTEC ROS 协议。

### 主机 → 底盘：11 字节命令帧

| 字节 | 含义 |
|---|---|
| 0 | 帧头 `7B` |
| 1 | `00` 速度指令；`10`（十六进制）释放本通道控制权 |
| 2 | 保留，固定 `00` |
| 3–4 | `vx`：有符号 16 位大端，mm/s |
| 5–6 | `vy`：有符号 16 位大端，必须为 0 |
| 7–8 | `wz`：有符号 16 位大端，mrad/s |
| 9 | 字节 0–8 逐字节 XOR |
| 10 | 帧尾 `7D` |

释放帧的三个速度字段必须全部为零；下表是十六进制**字节**，不是字符串：

```text
零速：      7B 00 00 00 00 00 00 00 00 7B 7D
前进 0.2：  7B 00 00 00 C8 00 00 00 00 B3 7D
释放控制权：7B 10 00 00 00 00 00 00 00 6B 7D
```

### 底盘 → 主机：22 字节遥测帧，约 20 Hz

```text
5A status 左轮H 左轮L 右轮H 右轮L 电压H 电压L
   加速度XH XL YH YL ZH ZL 角速度XH XL YH YL ZH ZL XOR A5
```

左右轮速度为有符号 16 位大端、单位 mm/s；电压为无符号 16 位大端、单位 mV；IMU 各轴为有符号 16 位大端的驱动计数。`XOR` 为前 20 字节异或。IMU 比例及轴定义见[串口控制说明](docs/串口控制说明.md)。

`status` 位：bit0 已解锁（ARMED），bit1 超时，bit2 接收错误，bit3 等待解锁或物理门控未就绪，bit4 输出停止，bit5 使能，bit6 低压，bit7 自检错误。已解锁和输出停止可以同时成立；输出停止不表示车轮已经机械静止。

### 底盘 → 主机：6 字节控制权帧

```text
5B 01 owner state XOR A6
```

`owner`：`00` 无控制方、`03` 串口 3、`04` 蓝牙；`state`：`00` 等零帧、`01` 已解锁、`02` 超时、`03` 物理失能、`04` 接收错误。`XOR` 为前 4 字节异或。遥测和控制权帧按字节流接收：一次读取可能是不完整帧或多个帧，须缓存、按帧头和长度解析并验证帧尾及校验。

### ROS 端控制约束

1. 上电等待控制板就绪后周期发送零速帧，取得控制权；只有回传确认 `owner=03`、已解锁、使能有效且无故障时才发送非零命令。蓝牙占权时不能抢占。
2. 建议以 20 Hz 发帧；底盘超过 500 ms 未收到有效命令会停止输出。ROS 桥接节点也须监控 `/cmd_vel`：建议超过 300 ms 无新消息就发零速，不能持续重发旧非零命令。
3. 通信异常、回执超时、失能或控制权丢失时丢弃旧速度；恢复后先重新零帧解锁，再等待新的运动命令。交给蓝牙时发送释放帧并停止串口心跳。
4. 速度发布节点示例、打包代码、接线和联调验收见 [Jetson NX ROS 2 速度控制开发参考](docs/Jetson_NX_ROS2_速度控制开发参考.md)；完整字段、安全行为见 [串口控制说明](docs/串口控制说明.md) 和 [蓝牙控制与客户端开发](docs/蓝牙控制与客户端开发.md)。ROS 2 节点由上位机负责人在其仓库实现。

## 上位机工具（`tools/`）

| 脚本 | 用途 |
|---|---|
| [serial_drive.py](tools/serial_drive.py) | 命令行发速度指令（`--vx/--wz`、`--interactive` WASD 遥控、`--dry-run`） |
| [chassis_web.py](tools/chassis_web.py) | 实时仪表盘；按住运行、松开停机，0.5秒操作/回执超时 |
| [serial_probe.py](tools/serial_probe.py) | 读串口原始字节，排查收发（DTR/RTS、波特率） |

```powershell
pip install pyserial
python tools\chassis_web.py --source uart3 --port COM3     # 浏览器打开 http://127.0.0.1:8000/
```

## 开发入口

| 入口 | 用途 |
|---|---|
| [USER/WHEELTEC.uvprojx](USER/WHEELTEC.uvprojx) | Keil工程，选择 `Diff_Car`；当前Compiler 5.06 update 5 |
| [chassis_config.h](CarType/Inc/chassis_config.h) | 底盘参数与方向配置 |
| `USER/` / `BALANCE/` / `HARDWARE/` | 初始化 / 控制与通信 / 硬件驱动 |
| [AGENTS.md](AGENTS.md) | 协作约定 |
| [底盘适配说明](docs/底盘适配说明.md) | 接线、参数依据与待验证项 |
| [串口控制说明](docs/串口控制说明.md) | UART 速度指令协议、安全语义与集成 |
| [Jetson NX ROS 2 速度控制开发参考](docs/Jetson_NX_ROS2_速度控制开发参考.md) | 上位机节点示例、协议约束与联调交接 |
| [CodeGraph工作流](docs/CodeGraph工作流.md) | 查询、影响分析与索引同步 |

技术栈：C＋STM32标准外设库＋FreeRTOS，Keil工程。

```powershell
codegraph sync .  # 首次使用：codegraph init --yes .
python tests/test_chassis_profile.py
python tests/test_chassis_uart_control.py
python tests/test_chassis_serial_safety.py
python tests/test_chassis_telemetry.py
python tests/test_chassis_web.py
python tests/test_bluetooth_control.py
python tests/test_bluetooth_tx.py
```

## 诊断测试与仓库边界

日常使用 `Diff_Car`；PC8、单轮和两轮闭环诊断目标集中见 [诊断测试入口](docs/diagnostics/README.md)。这些目标保留排障用途，不与正式串口程序混用。

`OBJ/`、HEX、CodeGraph索引及个人工具状态不提交。源码备份由Git历史保存，不保留 `.orig` / `.bak` 副本。原厂驱动库、其他车型、测试与必要硬件手册保留；Keil工程和本地编译产生的烧录文件仍按原流程使用。
