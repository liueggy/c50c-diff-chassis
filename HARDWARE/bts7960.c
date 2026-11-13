#include "bts7960.h"

/* Lift platform motor driver for the BTS7960 (IBT-2) module.
 * Level control only - see header for wiring and state table. */

static Lift_State_t lift_state = LIFT_BRAKE;

static void lift_apply(Lift_State_t state)
{
    switch (state)
    {
    case LIFT_UP:
        LIFT_UP_MASK();
        lift_state = LIFT_UP;
        break;
    case LIFT_DOWN:
        LIFT_DOWN_MASK();
        lift_state = LIFT_DOWN;
        break;
    case LIFT_BRAKE: /* both high: dynamic brake */
        LIFT_RPWM = 1;
        LIFT_LPWM = 1;
        lift_state = LIFT_BRAKE;
        break;
    case LIFT_FREE: /* both low: coast */
    default:
        LIFT_RPWM = 0;
        LIFT_LPWM = 0;
        lift_state = LIFT_FREE;
        break;
    }
}

void LiftMotor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    ENABLE_LIFT_PIN_CLOCK;

    /* Push-pull output, reset level low: the pair is guaranteed low the
     * moment the pins leave their reset state, so the motor cannot be
     * driven before LiftMotor_Init() runs. */
    GPIO_InitStructure.GPIO_Pin   = LIFT_RPWM_PIN | LIFT_LPWM_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; /* logic level only */
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(LIFT_RPWM_PORT, &GPIO_InitStructure);

    /* Boot state: dynamic brake. Holding the platform requires no motion
     * and is the safest state against self-weight sag. */
    LIFT_RPWM = 0;
    LIFT_LPWM = 0;
    lift_apply(LIFT_BRAKE);
}

void LiftMotor_Set(Lift_State_t state)
{
    /* Only enumerated states are accepted; anything else is ignored so a
     * corrupted command from the control layer cannot energize the motor
     * in an undefined way. */
    if (state == LIFT_FREE || state == LIFT_UP ||
        state == LIFT_DOWN || state == LIFT_BRAKE)
    {
        lift_apply(state);
    }
}

Lift_State_t LiftMotor_Get(void)
{
    return lift_state;
}

void LiftMotor_Brake(void)
{
    lift_apply(LIFT_BRAKE);
}

void LiftMotor_Free(void)
{
    lift_apply(LIFT_FREE);
}
