#ifndef __CHASSIS_UART_CONTROL_H
#define __CHASSIS_UART_CONTROL_H

#include <stdint.h>

#define CHASSIS_UART_FRAME_SIZE                 11U
#define CHASSIS_UART_RX_RING_SIZE               128U
#define CHASSIS_UART_INTERBYTE_TIMEOUT_MS       50U
#define CHASSIS_UART_COMMAND_TIMEOUT_MS         500U
#define CHASSIS_UART_MAX_LINEAR_MM_S            700
#define CHASSIS_UART_MAX_ANGULAR_MRAD_S         1000
#define CHASSIS_UART_MAX_LINEAR_M_S             0.7f
#define CHASSIS_UART_MAX_ANGULAR_RAD_S          1.0f

#define CHASSIS_UART_STATUS_WAIT_ZERO           0U
#define CHASSIS_UART_STATUS_ACTIVE              1U
#define CHASSIS_UART_STATUS_TIMEOUT             2U
#define CHASSIS_UART_STATUS_PHYSICAL_DISABLED   3U
#define CHASSIS_UART_STATUS_RX_ERROR            4U

#define CHASSIS_COMMAND_RELEASE                   0x10U
#define CHASSIS_SOURCE_NONE                       0U
#define CHASSIS_SOURCE_UART3                      3U
#define CHASSIS_SOURCE_BLUETOOTH                  4U

void ChassisUart_Reset(void);
void ChassisUart_PushByte(uint8_t byte, uint32_t now_ms);
void ChassisUart_RxError(void);
uint8_t ChassisUart_Step(uint32_t now_ms, uint8_t physical_allowed,
                      float *vx, float *wz);
uint8_t ChassisUart_Status(void);

void ChassisBluetooth_PushByte(uint8_t byte, uint32_t now_ms);
void ChassisBluetooth_RxError(void);
uint8_t ChassisControl_Step(uint32_t now_ms, uint8_t physical_allowed, float *vx, float *wz);
uint8_t ChassisControl_Status(void);
uint8_t ChassisControl_Source(void);
/* Display-only UART3 valid-command activity, 1 second grace period. */
uint8_t ChassisUart_LinkOnline(uint32_t now_ms);

#endif
