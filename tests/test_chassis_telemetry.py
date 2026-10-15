"""Golden-frame tests of the actual 22-byte telemetry packing block."""
import subprocess
import tempfile
from pathlib import Path
from test_chassis_profile import ROOT, function


def main():
    source = (ROOT / "BALANCE/data_task.c").read_bytes().decode("gb18030").replace("\r\n", "\n")
    start = source.index("\t{\n\t\tint ct_lA")
    opening = source.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    block = source[start:end]
    checksum = function(source, "uint8_t Check_BCC(const uint8_t *data, uint16_t length)")
    harness = r'''
#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "chassis_uart_control.h"
#define CHASSIS_TELEM_HEADER 0x5a
#define CHASSIS_TELEM_TAIL 0xa5
static struct { float Encoder; } motor_a,motor_b;
static struct {
    struct {float Encoder;} MOTOR_A,MOTOR_B;
    float voltage; int LowPower;
} robot;
static struct {int x,y,z;} accel,gyro;
static struct {struct {int x,y,z;} accel,gyro;} imu;
static struct {int FlagStop;} robot_control;
static struct {int errorflag;} robot_check;
static int EN=1;
static uint8_t status=CHASSIS_UART_STATUS_ACTIVE;
uint8_t ChassisUart_Status(void) {return status;}
static unsigned char chassis_telem[22];
CHECKSUM
static void pack(void) BLOCK
int main(void) {
    robot.MOTOR_A.Encoder=-0.2f; robot.MOTOR_B.Encoder=0.35f; robot.voltage=12.19f;
    imu.accel.x=12; imu.accel.y=-8; imu.accel.z=16384;
    imu.gyro.x=131; imu.gyro.y=-65; imu.gyro.z=655;
    robot_control.FlagStop=1;
    pack();
    /* ARMED + OUTPUT_STOPPED + ENABLED, including a nonzero gyro while stopped. */
    const uint8_t expected[20]={0x5a,0x31,0xff,0x38,0x01,0x5e,0x2f,0x9e,
       0xff,0xf8,0xff,0xf4,0x40,0,0xff,0xbf,0xff,0x7d,0x02,0x8f};
    assert(memcmp(chassis_telem,expected,20)==0);
    uint8_t sum=0; for(int i=0;i<20;i++) sum^=expected[i];
    assert(chassis_telem[20]==sum && chassis_telem[21]==0xa5);
    status=CHASSIS_UART_STATUS_TIMEOUT; pack(); assert(chassis_telem[1]==0x32);
    status=CHASSIS_UART_STATUS_RX_ERROR; EN=0; robot.LowPower=1; robot_check.errorflag=1;
    pack(); assert(chassis_telem[1]==0xd4);
    puts("PASS 22-byte telemetry golden frame and ARMED/STOP/IMU semantics");
}
'''.replace("CHECKSUM", checksum).replace("BLOCK", block)
    with tempfile.TemporaryDirectory(prefix="chassis-telemetry-") as directory:
        temp = Path(directory)
        (temp / "test.c").write_text(harness, encoding="utf-8")
        binary = temp / "test.exe"
        subprocess.run(["gcc", "-std=c99", "-I" + str(ROOT / "BALANCE/Inc"),
                        str(temp / "test.c"), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
