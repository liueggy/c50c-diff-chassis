"""Host tests of real profile selection, kinematics and motor output guards.

Run with Python and GCC on PATH; no embedded hardware is required.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def function(source, signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


STUB = r'''
#include <stdint.h>
#include "eggy_chassis_config.h"
typedef unsigned char u8;
typedef struct { float Target, Encoder; int Output; } Motor;
typedef struct {
    float WheelSpacing, AxleSpacing, GearRatio, Wheel_Circ, Encoder_precision;
    int EncoderAccuracy;
} Hardware;
typedef struct {
    Hardware HardwareParam; Motor MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
    int type, V_KP, V_KI; float voltage;
} ROBOT_t;
extern ROBOT_t robot;
#define CAR_NUMBER 9
#define CarMode_Ch 12
#define TOP_DIFF_wheelspacing 0.329f
#define TOP_DIFF_WheelDiameter 0.125f
#define FOUR_WHEEL_DIFF_BS_wheelspacing 0.573f
#define FOUR_WHEEL_DIFF_DL_wheelspacing 0.605f
#define FOUR_WHEEL_DIFF_WheelDiameter 0.254f
#define MD36N_27 27
#define MD36N_51 51
#define MD60N_18 18
#define MD60N_47 47
#define GMR_500 500
#define VEL_KP 300
#define VEL_KI 300
#define EncoderMultiples 4
#define PI 3.14159265358979323846f
int Get_ADC1_Average(int channel, int samples);
void Robot_Select(void);
'''

HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include "robot_init.h"
static int adc;
int Get_ADC1_Average(int channel, int samples) { return adc; }
static struct { int HardWare_Ver, Time_count; } SysVal = {2, 0};
static struct { int AllowRecharge; } charger;
static struct { int errorflag; } robot_check;
static struct {
    int SoftWare_Stop, FlagStop; float Vx, Vy, Vz, smooth_Vx, smooth_Vy, smooth_Vz;
} robot_control;
static int EN=1, AIN1, AIN2, BIN1, BIN2, V1_0_BIN1, V1_0_BIN2, PWMA, PWMB, PWMC, PWMD, CIN1, CIN2, DIN1, DIN2;
#define V1_0 0
#define V1_1 1
FUNCTIONS
int main(void) {
    adc=0; Robot_Select();
#if EGGY_CHASSIS_ENABLED
    assert(robot.type==0);
    assert(robot.HardwareParam.Encoder_precision==56000.0f);
    assert(fabsf(robot.HardwareParam.Wheel_Circ-0.2032f*PI)<0.000001f);
    adc=4095; Robot_Select();
    assert(robot.type==0); /* The physical TYPE knob cannot select another profile. */
#else
    assert(robot.HardwareParam.Encoder_precision==54000.0f);
    adc=455; Robot_Select(); assert(robot.type==1);
    assert(robot.HardwareParam.Encoder_precision==102000.0f);
    adc=4095; Robot_Select(); assert(robot.type==0);
    assert(robot.HardwareParam.Encoder_precision==54000.0f);
#endif
    InverseKinematics_diff(0.2f,0.0f);
    assert(robot.MOTOR_A.Target==robot.MOTOR_B.Target);
    InverseKinematics_diff(0.0f,0.5f);
    assert(robot.MOTOR_A.Target<0.0f && robot.MOTOR_B.Target>0.0f);
#if EGGY_CHASSIS_ENABLED
    if (EGGY_TRACK_WIDTH_M > 0.0f) {
        assert(fabsf(robot.HardwareParam.WheelSpacing-EGGY_TRACK_WIDTH_M)<0.000001f);
        assert(fabsf(robot.MOTOR_A.Target+0.25f*EGGY_TRACK_WIDTH_M)<0.000001f);
        assert(fabsf(robot.MOTOR_B.Target-0.25f*EGGY_TRACK_WIDTH_M)<0.000001f);
    }
#endif
    robot.voltage=24.0f;
#if EGGY_CHASSIS_ENABLED && EGGY_MOTOR_A_PROBE
    /* Diagnostic bypass must remain bounded even though normal READY is false. */
    assert(Turn_Off()==1);
    Set_Pwm(5000,5000,5000,5000,1500);
    assert(PWMA==1680 && PWMB==0 && PWMC==0 && PWMD==0);
    assert(AIN1==0 && AIN2==1 && BIN1==0 && BIN2==0);
    Set_Pwm(-5000,5000,0,0,0); assert(PWMA==0 && AIN2==0);
    EN=0; Set_Pwm(1680,0,0,0,0); assert(PWMA==0); EN=1;
    robot.voltage=19.0f; Set_Pwm(1680,0,0,0,0); assert(PWMA==0);
    robot.voltage=24.0f;
    robot_control.SoftWare_Stop=1; Set_Pwm(1680,0,0,0,0); assert(PWMA==0);
    robot_control.SoftWare_Stop=0;
    SysVal.Time_count=499; Eggy_MotorA_ProbeStep(); assert(PWMA==0);
    SysVal.Time_count=500; Eggy_MotorA_ProbeStep(); assert(PWMA==0);
    SysVal.Time_count=525; Eggy_MotorA_ProbeStep(); assert(PWMA==840);
    SysVal.Time_count=550; Eggy_MotorA_ProbeStep(); assert(PWMA==1680);
    SysVal.Time_count=775; Eggy_MotorA_ProbeStep(); assert(PWMA==840);
    SysVal.Time_count=800; Eggy_MotorA_ProbeStep(); assert(PWMA==0);
    SysVal.Time_count=600; EN=0; Eggy_MotorA_ProbeStep(); assert(PWMA==0);
    EN=1; SysVal.Time_count=601; Eggy_MotorA_ProbeStep(); assert(PWMA==0);
    return 0;
#else
    Set_Pwm(1200,-800,0,0,0);
    assert(Turn_Off()==EXPECT_BLOCKED);
    assert(PWMA==(EXPECT_BLOCKED ? 0 : 1200));
    assert(PWMB==(EXPECT_BLOCKED ? 0 : 800));
#if EGGY_CHASSIS_ENABLED && EGGY_PC8_PIN_TEST
    assert(AIN1==0 && AIN2==0 && BIN1==0 && BIN2==0);
    assert(CIN1==0 && CIN2==0 && DIN1==0 && DIN2==0);
    assert(PWMC==0 && PWMD==0);
#endif
    EN=0; assert(Turn_Off()==1); EN=1;
    robot.voltage=19.0f; assert(Turn_Off()==1); robot.voltage=24.0f;
    robot_control.SoftWare_Stop=1; assert(Turn_Off()==1);
    return 0;
#endif
}
'''


def main():
    gcc = shutil.which("gcc")
    if not gcc:
        raise SystemExit("GCC is required on PATH for host tests.")
    balance = (ROOT / "BALANCE/balance_task.c").read_bytes().decode("gb18030")
    functions = "\n".join(function(balance, signature) for signature in (
        "static void InverseKinematics_diff(float Vx,float Vz)",
        "static void Set_Pwm(int m_a,int m_b,int m_c,int m_d,int servo)",
        "static u8 Turn_Off(void)",
    ))
    functions += "\n#if EGGY_CHASSIS_ENABLED && EGGY_MOTOR_A_PROBE\n" + function(
        balance, "static void Eggy_MotorA_ProbeStep(void)") + "\n#endif\n"
    cases = (
        ("unconfigured", [], 1),
        ("confirmation_without_width", ["EGGY_TRACK_WIDTH_M=0.0f", "EGGY_CONFIG_CONFIRMED=1"], 1),
        ("width_without_confirmation", ["EGGY_TRACK_WIDTH_M=0.55f"], 1),
        ("configured", ["EGGY_TRACK_WIDTH_M=0.55f", "EGGY_CONFIG_CONFIRMED=1"], 0),
        ("measured_default_confirmed", ["EGGY_CONFIG_CONFIRMED=1"], 0),
        ("vendor_fallback", ["EGGY_CHASSIS_ENABLED=0"], 0),
        ("pc8_test_blocks_confirmed", ["EGGY_PC8_PIN_TEST=1", "EGGY_CONFIG_CONFIRMED=1"], 1),
        ("motor_a_probe_bounded", ["EGGY_MOTOR_A_PROBE=1"], 1),
        ("motor_a_probe_confirmed", ["EGGY_MOTOR_A_PROBE=1", "EGGY_CONFIG_CONFIRMED=1"], 1),
    )
    # Zero width tests still exercise the gate; the default is the measured 0.55 m.
    with tempfile.TemporaryDirectory(prefix="eggy-tests-") as temporary:
        temp = Path(temporary)
        (temp / "robot_init.h").write_text(STUB, encoding="ascii")
        (temp / "profile.c").write_text(
            (ROOT / "CarType/diff_robot_init.c").read_bytes().decode("gb18030"), encoding="utf-8")
        (temp / "harness.c").write_text(HARNESS.replace("FUNCTIONS", functions), encoding="utf-8")
        for name, definitions, blocked in cases:
            binary = temp / (name + ".exe")
            subprocess.run([gcc, "-std=c99", "-DDIFF_CAR", f"-DEXPECT_BLOCKED={blocked}",
                            *["-D" + value for value in definitions],
                            "-I" + str(temp), "-I" + str(ROOT / "CarType/Inc"),
                            str(temp / "profile.c"), str(temp / "harness.c"),
                            "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
            print("PASS", name)


if __name__ == "__main__":
    main()
