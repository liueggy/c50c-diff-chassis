#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* Only Diff_Car selects this chassis. Other targets keep the vendor profile. */
#ifndef CHASSIS_ENABLED
#if defined DIFF_CAR
#define CHASSIS_ENABLED 1
#else
#define CHASSIS_ENABLED 0
#endif
#endif

/* Diagnostic mode is enabled only by the Diff_Car_PC8_Test target. */
#ifndef CHASSIS_PC8_PIN_TEST
#define CHASSIS_PC8_PIN_TEST 0
#endif

/* Only the explicit MotorA test target enables this open-loop pulse. */
#ifndef CHASSIS_MOTOR_A_PROBE
#define CHASSIS_MOTOR_A_PROBE 0
#endif
#ifndef CHASSIS_MOTOR_B_PROBE
#define CHASSIS_MOTOR_B_PROBE 0
#endif
#if CHASSIS_MOTOR_A_PROBE && CHASSIS_MOTOR_B_PROBE
#error "Select only one motor probe channel"
#endif
#define CHASSIS_MOTOR_A_PROBE_PWM 8400 /* 50 percent of the 16800-count period. */
#define CHASSIS_MOTOR_A_PROBE_MIN_VOLTAGE 10.0f /* Bench test with a 12 V supply. */
#if CHASSIS_PC8_PIN_TEST && (CHASSIS_MOTOR_A_PROBE || CHASSIS_MOTOR_B_PROBE)
#error "PC8 pin test and motor probe cannot be enabled together"
#endif

/* Explicit finite two-wheel closed-loop commissioning target. */
#ifndef CHASSIS_CLOSED_LOOP_TEST
#define CHASSIS_CLOSED_LOOP_TEST 0
#endif
#if CHASSIS_CLOSED_LOOP_TEST && (CHASSIS_PC8_PIN_TEST || CHASSIS_MOTOR_A_PROBE || CHASSIS_MOTOR_B_PROBE)
#error "Select only one commissioning target"
#endif
#define CHASSIS_CLOSED_LOOP_SPEED 0.08f
#define CHASSIS_CLOSED_LOOP_MAX_PWM 8400

/* Open-loop duty test: both wheels forward at a fixed PWM, so the encoder
   feedback (max speed) at that duty can be read on the OLED (DL/DR lines). */
#ifndef CHASSIS_OPEN_LOOP_TEST
#define CHASSIS_OPEN_LOOP_TEST 0
#endif
#if CHASSIS_OPEN_LOOP_TEST && (CHASSIS_PC8_PIN_TEST || CHASSIS_MOTOR_A_PROBE || CHASSIS_MOTOR_B_PROBE || CHASSIS_CLOSED_LOOP_TEST)
#error "Select only one commissioning target"
#endif
#define CHASSIS_OPEN_LOOP_PWM 16800 /* 100% of the 16800-count period. */

/* Set these only after measuring geometry and checking wiring/encoder supply. */
#ifndef CHASSIS_CONFIG_CONFIRMED
#define CHASSIS_CONFIG_CONFIRMED 1
#endif
#ifndef CHASSIS_TRACK_WIDTH_M
#define CHASSIS_TRACK_WIDTH_M 0.55f /* Measured wheel-center spacing: 550 mm. */
#endif

/* Fixed on-board supply of at most 12 V: lower the vendor ~24 V thresholds. */
#ifndef CHASSIS_STOP_VOLTAGE_V
#define CHASSIS_STOP_VOLTAGE_V 9.5f /* Motor output disabled below this (non-recharge). */
#endif
#ifndef CHASSIS_LOW_VOLTAGE_V
#define CHASSIS_LOW_VOLTAGE_V 9.5f /* Low-battery flag set below this. */
#endif
#ifndef CHASSIS_LOW_CLEAR_VOLTAGE_V
#define CHASSIS_LOW_CLEAR_VOLTAGE_V 10.5f /* Low-battery flag cleared above this. */
#endif

/* Nominal values from the old chassis manuals; verify against the real motor. */
#define CHASSIS_WHEEL_DIAMETER_M 0.2032f
#define CHASSIS_GEAR_RATIO 28.0f
#define CHASSIS_ENCODER_LINES 500
#define CHASSIS_VELOCITY_KP 300
#define CHASSIS_VELOCITY_KI 300

/* A is left, B is right. Motor directions and encoder signs verified on the installed chassis. */
#define CHASSIS_MOTOR_A_SIGN (-1)
#define CHASSIS_MOTOR_B_SIGN (-1)
#define CHASSIS_ENCODER_A_SIGN 1
#define CHASSIS_ENCODER_B_SIGN (-1)

/* UART4 Bluetooth module must already use this serial baud rate. */
#ifndef CHASSIS_BLUETOOTH_BAUD
#define CHASSIS_BLUETOOTH_BAUD 9600
#endif

/* Normal motion uses the arbitrated UART3/UART4 link; diagnostics bypass it. */
#ifndef CHASSIS_SERIAL_CONTROL
#define CHASSIS_SERIAL_CONTROL (CHASSIS_ENABLED && !CHASSIS_PC8_PIN_TEST && !CHASSIS_MOTOR_A_PROBE && !CHASSIS_MOTOR_B_PROBE && !CHASSIS_CLOSED_LOOP_TEST && !CHASSIS_OPEN_LOOP_TEST)
#endif

/* Chinese 2-page OLED UI replaces the vendor pages for normal operation. */
#ifndef CHASSIS_CN_UI
#define CHASSIS_CN_UI (CHASSIS_ENABLED && !CHASSIS_PC8_PIN_TEST && !CHASSIS_MOTOR_A_PROBE && !CHASSIS_MOTOR_B_PROBE && !CHASSIS_CLOSED_LOOP_TEST)
#endif

#define CHASSIS_READY() (!CHASSIS_PC8_PIN_TEST && !CHASSIS_MOTOR_A_PROBE && !CHASSIS_MOTOR_B_PROBE && !CHASSIS_CLOSED_LOOP_TEST && CHASSIS_CONFIG_CONFIRMED && \
    CHASSIS_TRACK_WIDTH_M > 0.0f && CHASSIS_WHEEL_DIAMETER_M > 0.0f && \
    CHASSIS_GEAR_RATIO > 0.0f && CHASSIS_ENCODER_LINES > 0)

/* The installed two-channel PWM driver has no D50A RS485 management port. */
#define CHASSIS_USE_D50A (!CHASSIS_ENABLED)

#endif
