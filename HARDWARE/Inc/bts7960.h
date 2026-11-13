#ifndef __BTS7960_H
#define __BTS7960_H

#include "sys.h"

/* Lift platform DC motor driven by one BTS7960 (IBT-2) H-bridge module.
 *
 * Wiring (bottom board header, see repo issue #2):
 *   RPWM = D8 (PD8), LPWM = D9 (PD9)
 *   R_EN + L_EN shorted together, tied to on-board 5V (always enabled)
 *   VCC = 5V, GND common ground; R_IS / L_IS not connected.
 *
 * Control (no PWM, level control only):
 *   RPWM=1 LPWM=0 -> up
 *   RPWM=0 LPWM=1 -> down
 *   both high     -> dynamic brake (parking default; also on comms loss)
 *   both low      -> free coast
 *
 * Note: PD11 is taken by the buzzer; PD8/PD9 are free (UART3 uses PB10/PB11,
 * CAN1 uses PD0/PD1). The 74HC244 buffer on the module runs at 5V, so the
 * 3.3V MCU levels are marginally high - verify level flips on the bench
 * before soldering. To add speed control later, move the shorted EN pair
 * from 5V to PB4 (TIM3_CH1 PWM); the GPIO wiring below stays unchanged.
 */

typedef enum
{
    LIFT_FREE  = 0, /* both outputs low: coast */
    LIFT_UP    = 1, /* RPWM high: platform up */
    LIFT_DOWN  = 2, /* LPWM high: platform down */
    LIFT_BRAKE = 3  /* both outputs high: dynamic brake */
} Lift_State_t;

/*--------BTS7960 (lift motor) config--------*/
#define ENABLE_LIFT_PIN_CLOCK   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE)

#define LIFT_RPWM_PORT          GPIOD
#define LIFT_RPWM_PIN           GPIO_Pin_8   /* D8 on the header */
#define LIFT_RPWM               PDout(8)

#define LIFT_LPWM_PORT          GPIOD
#define LIFT_LPWM_PIN           GPIO_Pin_9   /* D9 on the header */
#define LIFT_LPWM               PDout(9)

/* Direction macros: swap these two lines if up/down wiring differs. */
#define LIFT_UP_MASK()      (LIFT_RPWM = 1, LIFT_LPWM = 0)
#define LIFT_DOWN_MASK()    (LIFT_RPWM = 0, LIFT_LPWM = 1)

/*------------------------------------------*/

/* Init both pins as push-pull outputs, boot state = dynamic brake
 * (platform held; brake is motionless and therefore safe at boot). */
void LiftMotor_Init(void);

/* Set the lift motor state. Invalid values are ignored (fail-safe). */
void LiftMotor_Set(Lift_State_t state);

/* Last commanded state (not a feedback of the driver itself). */
Lift_State_t LiftMotor_Get(void);

/* Convenience wrappers for the control layer. */
void LiftMotor_Brake(void); /* parking / comms loss / low battery */
void LiftMotor_Free(void);  /* coast, for maintenance */

#endif
