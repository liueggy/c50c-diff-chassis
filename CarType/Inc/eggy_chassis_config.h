#ifndef EGGY_CHASSIS_CONFIG_H
#define EGGY_CHASSIS_CONFIG_H

/* Only Diff_Car selects this chassis. Other targets keep the vendor profile. */
#ifndef EGGY_CHASSIS_ENABLED
#if defined DIFF_CAR
#define EGGY_CHASSIS_ENABLED 1
#else
#define EGGY_CHASSIS_ENABLED 0
#endif
#endif

/* Diagnostic mode is enabled only by the Diff_Car_PC8_Test target. */
#ifndef EGGY_PC8_PIN_TEST
#define EGGY_PC8_PIN_TEST 0
#endif

/* Only the explicit MotorA test target enables this open-loop pulse. */
#ifndef EGGY_MOTOR_A_PROBE
#define EGGY_MOTOR_A_PROBE 0
#endif
#ifndef EGGY_MOTOR_B_PROBE
#define EGGY_MOTOR_B_PROBE 0
#endif
#if EGGY_MOTOR_A_PROBE && EGGY_MOTOR_B_PROBE
#error "Select only one motor probe channel"
#endif
#define EGGY_MOTOR_A_PROBE_PWM 8400 /* 50 percent of the 16800-count period. */
#define EGGY_MOTOR_A_PROBE_MIN_VOLTAGE 10.0f /* Bench test with a 12 V supply. */
#if EGGY_PC8_PIN_TEST && (EGGY_MOTOR_A_PROBE || EGGY_MOTOR_B_PROBE)
#error "PC8 pin test and motor probe cannot be enabled together"
#endif

/* Explicit finite two-wheel closed-loop commissioning target. */
#ifndef EGGY_CLOSED_LOOP_TEST
#define EGGY_CLOSED_LOOP_TEST 0
#endif
#if EGGY_CLOSED_LOOP_TEST && (EGGY_PC8_PIN_TEST || EGGY_MOTOR_A_PROBE || EGGY_MOTOR_B_PROBE)
#error "Select only one commissioning target"
#endif
#define EGGY_CLOSED_LOOP_SPEED 0.08f
#define EGGY_CLOSED_LOOP_MAX_PWM 8400

/* Set these only after measuring geometry and checking wiring/encoder supply. */
#ifndef EGGY_CONFIG_CONFIRMED
#define EGGY_CONFIG_CONFIRMED 0
#endif
#ifndef EGGY_TRACK_WIDTH_M
#define EGGY_TRACK_WIDTH_M 0.55f /* Measured wheel-center spacing: 550 mm. */
#endif

/* Nominal values from the old chassis manuals; verify against the real motor. */
#define EGGY_WHEEL_DIAMETER_M 0.2032f
#define EGGY_GEAR_RATIO 28.0f
#define EGGY_ENCODER_LINES 500
#define EGGY_VELOCITY_KP 300
#define EGGY_VELOCITY_KI 300

/* A is left, B is right. Motor directions and encoder signs verified on the installed chassis. */
#define EGGY_MOTOR_A_SIGN (-1)
#define EGGY_MOTOR_B_SIGN (-1)
#define EGGY_ENCODER_A_SIGN 1
#define EGGY_ENCODER_B_SIGN (-1)

#define EGGY_CHASSIS_READY() (!EGGY_PC8_PIN_TEST && !EGGY_MOTOR_A_PROBE && !EGGY_MOTOR_B_PROBE && !EGGY_CLOSED_LOOP_TEST && EGGY_CONFIG_CONFIRMED && \
    EGGY_TRACK_WIDTH_M > 0.0f && EGGY_WHEEL_DIAMETER_M > 0.0f && \
    EGGY_GEAR_RATIO > 0.0f && EGGY_ENCODER_LINES > 0)

/* The installed two-channel PWM driver has no D50A RS485 management port. */
#define EGGY_USE_D50A (!EGGY_CHASSIS_ENABLED)

#endif
