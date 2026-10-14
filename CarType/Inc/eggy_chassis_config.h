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

/* A is left, B is right. Signs are provisional C50C defaults, not measured. */
#define EGGY_MOTOR_A_SIGN 1
#define EGGY_MOTOR_B_SIGN 1
#define EGGY_ENCODER_A_SIGN 1
#define EGGY_ENCODER_B_SIGN (-1)

#define EGGY_CHASSIS_READY() (EGGY_CONFIG_CONFIRMED && \
    EGGY_TRACK_WIDTH_M > 0.0f && EGGY_WHEEL_DIAMETER_M > 0.0f && \
    EGGY_GEAR_RATIO > 0.0f && EGGY_ENCODER_LINES > 0)

/* The installed two-channel PWM driver has no D50A RS485 management port. */
#define EGGY_USE_D50A (!EGGY_CHASSIS_ENABLED)

#endif
