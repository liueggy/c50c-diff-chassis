# Jetson NX ROS 2 底盘速度控制开发参考

> 交接对象：在 Jetson NX 上开发 ROS 2 功能的人员或 AI 助手。本文以 C50C V4.3 差速底盘当前固件、串口协议 v1.2 为依据。先核实实车烧录版本、Jetson 系统和 ROS 2 版本；本文不是实车联调通过的证明。

## 1. 开发目标与边界

在 Jetson 的 ROS 2 工作空间实现：速度指令发布端 → `/cmd_vel` → 串口桥接节点 → C50C USART3。发布端可来自遥控、导航或测试节点；桥接节点独占串口，负责协议编解码、控制权、安全超时和回传。本固件仓库只提供协议和参考，不要求把 ROS 节点放入此仓库。不要将 ROS 消息直接原样写入单片机，也不要默认原厂 WHEELTEC 协议兼容。

开始前检查 JetPack/Ubuntu、已安装 ROS 2 发行版、Python 及依赖、工作空间和现有节点；通过实际设备和接线确定串口路径，不假定 `/dev/ttyUSB0` 或 `/dev/ttyTHS1`。检查设备权限、占用、串口服务和电平。非必要不要升级系统、切换 ROS 版本或修改固件。用户的导航栈若使用 `TwistStamped`，确认实际话题类型和 QoS，再通过适配器提取 `twist`；不能假设所有版本默认类型一致。

## 2. 示例：零速度发布节点

下面是独立 Python 示例，可保存为 `velocity_example.py`，在已 source ROS 2 环境中用 `python3 velocity_example.py` 运行。它持续发布零速度用于话题联调；它不连接串口，也不代表底盘已经运动。

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityExample(Node):
    def __init__(self):
        super().__init__('velocity_example')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        message = Twist()
        message.linear.x = 0.0    # m/s，正数前进
        message.angular.z = 0.0   # rad/s，正数左转
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = VelocityExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 正常退出时尽力发送零速；安全仍依赖桥接和底盘超时。
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

用 `ros2 topic info /cmd_vel -v` 确认消息类型、发布者、订阅者和 QoS，用 `ros2 topic echo /cmd_vel` 检查消息。在多个速度源并存时，必须由上位机实现明确的仲裁，避免相互覆盖；测试发布器不要与正式导航同时控制。

## 3. 接线和运动参数

- 用户说明当前使用控制板的 Type-C 串口 3 接口连接上位机。优先按该实际连接方式，将 Type-C 与 Jetson USB 连接；在 Jetson 上确认枚举出的串口设备，并通过收发和运动反馈核实它确实通往 USART3。不要假定设备名。
- 控制板 C50C V4.3，STM32F407VET6；运动接口 USART3：TX=PB10，RX=PB11，115200 baud、8N1、无硬件流控。上述引脚为 MCU 侧信号；使用板载 Type-C 时无需再按裸 TTL 引脚另行交叉接线。只有改用外置 TTL 转接时，才需核对 3.3 V 电平、TX/RX 交叉及共地。
- USART1 用于下载和状态回传，不接收该底盘运动命令；蓝牙 JDY-33 使用 UART4。用户指定的是 Type-C 串口 3，仍应实测核对所打开的设备对应正确接口。
- 左轮 A、右轮 B；轮距 0.55 m（用户测量）。ROS 机体 X 向前、Z 向上：`linear.x > 0` 前进，`angular.z > 0` 逆时针左转。现场再核对方向。
- 指令允许 `linear.x` 在 −0.7～0.7 m/s，`angular.z` 在 −1.0～1.0 rad/s，`linear.y` 必须为零；超限会使固件判错并解除控制。桥接端应在发送前验证有限数值、量程和未使用分量，非法输入安全置零并报告，不能发送截断或溢出值。
- 同时转弯时，轮目标为 `v_left = vx - wz * 0.55 / 2`、`v_right = vx + wz * 0.55 / 2`；它们可超过直行速度值。0.7 m/s 是协议直行指令上限，不是实车速度保证；约 0.95 m/s 为既往特定条件下的 100% 占空比观察，不是标定值。

## 4. Jetson → C50C：11 字节命令帧

| 字节 | 含义 |
|---|---|
| 0 | 帧头 `7B` |
| 1 | `00` 速度命令；`10`（十六进制）释放控制权 |
| 2 | 保留，必须为 `00` |
| 3–4 | `vx`，有符号 16 位大端，mm/s |
| 5–6 | `vy`，有符号 16 位大端，必须为 0 |
| 7–8 | `wz`，有符号 16 位大端，mrad/s |
| 9 | 字节 0～8 按位 XOR |
| 10 | 帧尾 `7D` |

速度单位换算：`vx_mm_s = linear.x * 1000`，`wz_mrad_s = angular.z * 1000`。采用明确的四舍五入策略，先验证范围再编码；发送的是二进制字节。释放帧所有速度字段必须为零。

```text
零速：    7B 00 00 00 00 00 00 00 00 7B 7D
前进0.2： 7B 00 00 00 C8 00 00 00 00 B3 7D
释放：    7B 10 00 00 00 00 00 00 00 6B 7D
```

打包参考（只负责格式，不包含串口管理或安全状态机）：

```python
import math
import struct


def make_speed_frame(vx_m_s: float, wz_rad_s: float) -> bytes:
    if not (math.isfinite(vx_m_s) and math.isfinite(wz_rad_s)):
        raise ValueError('non-finite velocity')
    if abs(vx_m_s) > 0.7 or abs(wz_rad_s) > 1.0:
        raise ValueError('velocity outside chassis limits')
    vx = round(vx_m_s * 1000)
    wz = round(wz_rad_s * 1000)
    frame = bytearray(b'\x7b\x00\x00')
    frame.extend(struct.pack('>hhh', vx, 0, wz))
    checksum = 0
    for byte in frame:
        checksum ^= byte
    frame.extend((checksum, 0x7d))
    return bytes(frame)


assert make_speed_frame(0.0, 0.0).hex() == '7b00000000000000007b7d'
assert make_speed_frame(0.2, 0.0).hex() == '7b000000c800000000b37d'
```

## 5. 底盘 → Jetson：两种回传帧

串口是字节流，每次读取可能得到半帧、多个帧或从中途开始。接收缓存按帧头寻找，长度不足则等待；帧尾、长度或 XOR 错误时丢一个字节重新同步。需要设置有界缓存，分别记录每类帧的最后有效时间。

**22 字节遥测帧**：`5A status 左轮速度 右轮速度 电压 加速度XYZ 角速度XYZ XOR A5`。速度、IMU 各分量为有符号 16 位大端；电压为无符号 16 位大端。字节 2–3 左轮速度、4–5 右轮速度（mm/s）；6–7 电压（mV）；8–13 加速度 XYZ，14–19 角速度 XYZ；20 是前 20 字节 XOR，21 是 `A5`。

状态字 bit0=ARMED，bit1=超时，bit2=接收错误，bit3=待解锁/物理门控未就绪，bit4=输出停止，bit5=使能，bit6=低压，bit7=自检错误。ARMED 和输出停止可同时为 1，表示零速待命；输出停止不等同车轮机械静止。

**6 字节控制权帧**：`5B 01 owner state XOR A6`，XOR 覆盖前 4 字节。`owner`：0 无、3 USART3、4 蓝牙；`state`：0 等零帧、1 已解锁、2 超时、3 物理失能、4 接收错误。示例：蓝牙持有 `5B 01 04 01 5F A6`。只在收到新鲜有效遥测和控制权帧、owner=3、state=1、ARMED 和使能有效且无故障时允许发送非零速度。

回传约 20 Hz；建议两种回传任一超过 0.5 秒未更新就清除旧目标并禁止运动。左右轮速度除以 1000 得 m/s；`vx=(v_left+v_right)/2`、`wz=(v_right-v_left)/0.55`。轮径/里程计需现场标定。IMU 数值及单位换算、坐标方向以《串口控制说明》为准；没有有效姿态时不要伪造四元数，ROS Imu 需标记姿态不可用。

## 6. 上位机安全与控制权状态机

1. 启动/串口重连先清空缓存与旧速度，等待底盘就绪，周期发送零帧申请控制。固件上电约前 10 秒忽略数据，约 12.3 秒后才可能完成门控；须以实际回传为准，不靠固定等待时间判定成功。
2. 建议按 20 Hz 发送。收到有效 owner=3、state=1 和正常遥测后进入就绪；只有此后收到的新鲜 `/cmd_vel` 才可产生非零输出。固件无有效命令达 500 ms 会停止输出并撤权。
3. 上游 `/cmd_vel` 超过 300 ms 未更新立即将目标设零，不得无限重复旧非零指令；使用单调时钟计算超时，校验异常值。串口错误、回传超时、使能关闭、故障或控制权丢失均丢弃旧速度，恢复后重新零帧申请并等待新指令。
4. ROS 与蓝牙互斥，不混合、不抢占。零帧可停机但继续持有控制权；要交给蓝牙时发送释放帧并停止 ROS 零帧/心跳，或关闭串口并等待底盘 500 ms 超时。不要在释放后立刻继续申请。
5. 串口写入设置有限超时，失败退出运动状态；尽力在正常关闭时发零速/释放，但不得依赖进程退出或系统信号一定会执行。避免发送队列积压及重连后重放旧命令。底盘超时只能保证电气输出被清除，实际停止距离需现场测量。

## 7. 联调验收与交付

依次进行并记录结果；先架空驱动轮，测试前确认电机使能开关和人工停止方式：

1. 离线单测打包：零、正负速度、边界、NaN、越界、校验；解析器测试半帧、粘帧、错帧、重同步、两类帧、控制权及超时。验证超时后不会重放旧速度。
2. 确认实际串口和 115200 参数；接收两类回传，持续零帧并确认 owner=3。屏幕“串口在线”只代表最近有有效串口指令，不代表取得控制权。
3. 架空低速测试前进、后退、原地左右转、零速；核对左右轮及速度符号。再测试上游消息中断、串口断开、使能 OFF、蓝牙占权和交接；确认停车及恢复须重新零帧和新命令。
4. 地面测试前检查场地、负载、电源和止车方式；记录目标速度、反馈、实际速度、停止距离、异常及改动。初次低速，逐步验证，不把软件测试当成实车结论。
5. 交付 ROS 工作空间中的源码、依赖与启动方法、串口参数、测试结果、实车记录和已知问题；说明 ROS 版本、实际设备路径和 `Twist` / `TwistStamped` 类型。没有验证的部分明确列为待测。

本文件依据同仓库的 [串口控制说明](串口控制说明.md)、[蓝牙控制与客户端开发](蓝牙控制与客户端开发.md) 和 [底盘适配说明](底盘适配说明.md)；如烧录固件版本或实际接线不同，以现场核实和对应源码为准。
