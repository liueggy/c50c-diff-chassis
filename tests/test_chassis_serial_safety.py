"""Run real UART parser, control gates and IRQ handlers without hardware."""
from pathlib import Path
import subprocess
import tempfile
from test_chassis_profile import ROOT, STUB, HARNESS, function


def main():
    source = (ROOT / "BALANCE/balance_task.c").read_bytes().decode("gb18030")
    prefix = HARNESS.split("FUNCTIONS")[0]
    prefix += r'''
#include "chassis_uart_control.h"
#define CONTROL_DELAY 1000
#define _ROS_Control 1
#define Set_Control_Mode(mode) ((void)(mode))
#define UN_LOCK 0
#define LOCK 1
typedef struct { float Bias, LastBias; int Output, kp, ki; } PI_CONTROLLER;
static PI_CONTROLLER PI_MotorA, PI_MotorB, PI_MotorC, PI_MotorD;
static uint8_t chassis_command_ready;
static float last_drive_x, last_drive_z;
static void Drive_Motor(float x,float y,float z) {last_drive_x=x;last_drive_z=z;(void)y;}
static void ResponseControl(void) {}
'''
    signatures = (
        "static uint8_t Chassis_PhysicalAllowed(void)",
        "static void Chassis_UpdateCommand(uint32_t now_ms)",
        "static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo)",
        "static void PI_Controller_Reset(PI_CONTROLLER *p)",
        "static void UnResponseControl(uint8_t mode)",
        "static u8 Turn_Off(void)",
    )
    functions = "\n".join(function(source, sig) for sig in signatures)
    test = r'''
static void send_frame(int vx, int wz, uint32_t now) {
    uint8_t frame[11] = {0x7b,0,0,0,0,0,0,0,0,0,0x7d};
    frame[3]=(uint16_t)vx>>8; frame[4]=vx;
    frame[7]=(uint16_t)wz>>8; frame[8]=wz;
    for (int i=0;i<9;i++) frame[9]^=frame[i];
    for (int i=0;i<11;i++) ChassisUart_PushByte(frame[i],now);
}
static void arm(uint32_t now) {
    send_frame(0,0,now); Chassis_UpdateCommand(now);
    assert(ChassisUart_Status()==CHASSIS_UART_STATUS_ACTIVE);
    assert(Turn_Off()==1); /* Armed zero never runs the PI. */
    send_frame(200,0,now+1); Chassis_UpdateCommand(now+1);
    assert(Turn_Off()==0 && robot_control.Vx>0.19f && last_drive_x>0.19f && last_drive_z==0);
}
int main(void) {
    Robot_Select(); robot.voltage=12.19f;
    ChassisUart_Reset(); SysVal.Time_count=1200;
    send_frame(0,0,1); Chassis_UpdateCommand(1);
    assert(!chassis_command_ready && Turn_Off()==1);
    SysVal.Time_count=1231; Chassis_UpdateCommand(2);
    send_frame(200,0,3); Chassis_UpdateCommand(3);
    assert(Turn_Off()==1); /* Fresh zero required after startup. */
    arm(10);
    Set_Pwm(-1200,-800,0,0,0); assert(PWMA==1200 && PWMB==800);
    EN=0; Chassis_UpdateCommand(12);
    assert(Turn_Off()==1);
    Set_Pwm(-1200,-800,0,0,0); assert(PWMA==0 && PWMB==0);
    send_frame(200,0,13); EN=1; Chassis_UpdateCommand(13);
    send_frame(200,0,14); Chassis_UpdateCommand(14);
    assert(Turn_Off()==1); /* Disabled queue and stale nonzero cannot restart. */
    arm(20);
    Chassis_UpdateCommand(521); assert(Turn_Off()==1);
    robot_control.smooth_Vx=0.7f; robot_control.smooth_Vz=1.0f;
    robot.MOTOR_A.Target=0.7f; robot.MOTOR_B.Target=0.7f;
    robot.MOTOR_A.Output=4000; PI_MotorA.Output=4000;
    UnResponseControl(UN_LOCK);
    assert(PWMA==0 && PWMB==0 && robot.MOTOR_A.Target==0);
    assert(robot_control.smooth_Vx==0 && robot_control.smooth_Vz==0 && PI_MotorA.Output==0);
    send_frame(200,0,522); Chassis_UpdateCommand(522); assert(Turn_Off()==1);
    arm(530); send_frame(0,0,532); Chassis_UpdateCommand(532);
    assert(Turn_Off()==1); /* Explicit zero stops immediately, while remaining armed. */
    arm(540); robot.voltage=9.4f; Chassis_UpdateCommand(542); assert(Turn_Off()==1);
    robot.voltage=12.19f; Chassis_UpdateCommand(543);
    send_frame(200,0,544); Chassis_UpdateCommand(544); assert(Turn_Off()==1);
    arm(550); robot_control.SoftWare_Stop=1; Chassis_UpdateCommand(552); assert(Turn_Off()==1);
    robot_control.SoftWare_Stop=0; Chassis_UpdateCommand(553); arm(560);
    robot.voltage=NAN; Chassis_UpdateCommand(562); assert(Turn_Off()==1);
    robot.voltage=29.0f; assert(!Chassis_PhysicalAllowed());
    robot.voltage=12.19f; robot_check.errorflag=2; assert(!Chassis_PhysicalAllowed());
    puts("PASS serial control safety integration");
}
'''
    with tempfile.TemporaryDirectory(prefix="chassis-safety-") as directory:
        temp = Path(directory)
        (temp / "robot_init.h").write_text(STUB, encoding="ascii")
        (temp / "profile.c").write_text(
            (ROOT / "CarType/diff_robot_init.c").read_bytes().decode("gb18030"), encoding="utf-8")
        (temp / "test.c").write_text("#include <stdio.h>\n" + prefix + functions + test, encoding="utf-8")
        binary = temp / "safety.exe"
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Wno-unused-variable", "-DDIFF_CAR", "-DCHASSIS_SERIAL_CONTROL=1",
                        "-I" + str(temp), "-I" + str(ROOT / "CarType/Inc"),
                        "-I" + str(ROOT / "BALANCE/Inc"), str(temp / "profile.c"),
                        str(temp / "test.c"), str(ROOT / "BALANCE/chassis_uart_control.c"),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    test_irq()


def test_irq():
    source = (ROOT / "BALANCE/uartx_callback.c").read_bytes().decode("gb18030")
    stubs = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#define CHASSIS_ENABLED 1
#define CONTROL_DELAY 1000
#define USART_FLAG_RXNE 0x20
#define USART_FLAG_ORE 0x08
#define USART_FLAG_NE 0x04
#define USART_FLAG_FE 0x02
#define USART_FLAG_PE 0x01
#define portTICK_PERIOD_MS 1
static struct { uint32_t SR, DR; } uart1, uart3;
#define USART1 (&uart1)
#define USART3 (&uart3)
static struct { int Time_count; } SysVal={1231};
static int pushed, errors;
static uint32_t xTaskGetTickCountFromISR(void) {return 77;}
static void ChassisUart_PushByte(uint8_t byte,uint32_t now) {
    assert(byte==0x7b && now==77); pushed++;
}
static void ChassisUart_RxError(void) {errors++;}
'''
    body = stubs + function(source, "int USART1_IRQHandler(void)") + "\n" + function(source, "int USART3_IRQHandler(void)")
    body += r'''
int main(void) {
    uart1.SR=USART_FLAG_RXNE|USART_FLAG_ORE; uart1.DR=0x7b;
    USART1_IRQHandler(); assert(pushed==0 && errors==0);
    uart3.SR=USART_FLAG_RXNE; uart3.DR=0x7b;
    USART3_IRQHandler(); assert(pushed==1 && errors==0);
    uart3.SR=USART_FLAG_ORE; USART3_IRQHandler(); assert(errors==1 && pushed==1);
    uart3.SR=USART_FLAG_RXNE|USART_FLAG_FE; USART3_IRQHandler(); assert(errors==2 && pushed==1);
    SysVal.Time_count=999; uart3.SR=USART_FLAG_RXNE;
    USART3_IRQHandler(); assert(pushed==1);
    puts("PASS UART3 exclusive ISR and hardware error handling");
}
'''
    with tempfile.TemporaryDirectory(prefix="chassis-irq-") as directory:
        temp = Path(directory)
        (temp / "irq.c").write_text(body, encoding="utf-8")
        binary = temp / "irq.exe"
        subprocess.run(["gcc", "-std=c99", str(temp / "irq.c"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
