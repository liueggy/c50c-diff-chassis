#include "chassis_uart_control.h"

#define CHASSIS_UART_RING_MASK (CHASSIS_UART_RX_RING_SIZE - 1U)

typedef struct
{
    uint8_t byte[CHASSIS_UART_FRAME_SIZE];
    uint32_t timestamp[CHASSIS_UART_FRAME_SIZE];
    uint8_t length;
} ChassisUartParser;

static volatile uint8_t chassis_rx_byte[CHASSIS_UART_RX_RING_SIZE];
static volatile uint32_t chassis_rx_timestamp[CHASSIS_UART_RX_RING_SIZE];
static volatile uint8_t chassis_rx_head;
static volatile uint8_t chassis_rx_tail;
static volatile uint8_t chassis_rx_error;

static ChassisUartParser chassis_parser;
static uint8_t chassis_armed;
static uint8_t chassis_status;
static uint8_t chassis_physical_allowed_previous;
static uint32_t chassis_last_valid_ms;
static float chassis_vx;
static float chassis_wz;

static uint32_t ChassisUart_Elapsed(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}

static void ChassisUart_ClearParser(void)
{
    chassis_parser.length = 0U;
}

static void ChassisUart_Disarm(uint8_t status)
{
    chassis_armed = 0U;
    chassis_vx = 0.0f;
    chassis_wz = 0.0f;
    chassis_status = status;
}

static void ChassisUart_FlushRing(void)
{
    chassis_rx_tail = chassis_rx_head;
}

static int16_t ChassisUart_ReadS16Be(const uint8_t *data)
{
    uint16_t value;

    value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    return (int16_t)value;
}

static void ChassisUart_ResyncWindow(void)
{
    uint8_t start;
    uint8_t index;
    uint8_t remaining;

    for (start = 1U; start < chassis_parser.length; ++start)
    {
        if (chassis_parser.byte[start] == 0x7BU)
        {
            break;
        }
    }

    if (start >= chassis_parser.length)
    {
        ChassisUart_ClearParser();
        return;
    }

    remaining = (uint8_t)(chassis_parser.length - start);
    for (index = 0U; index < remaining; ++index)
    {
        chassis_parser.byte[index] = chassis_parser.byte[start + index];
        chassis_parser.timestamp[index] = chassis_parser.timestamp[start + index];
    }
    chassis_parser.length = remaining;
}

static uint8_t ChassisUart_Checksum(const uint8_t *frame)
{
    uint8_t checksum;
    uint8_t index;

    checksum = 0U;
    for (index = 0U; index < 9U; ++index)
    {
        checksum ^= frame[index];
    }
    return checksum;
}

static void ChassisUart_AcceptFrame(uint32_t now_ms)
{
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t wz_mrad_s;
    uint32_t received_ms;

    received_ms = chassis_parser.timestamp[CHASSIS_UART_FRAME_SIZE - 1U];
    if (ChassisUart_Elapsed(now_ms, received_ms) >= CHASSIS_UART_COMMAND_TIMEOUT_MS)
    {
        ChassisUart_Disarm(CHASSIS_UART_STATUS_TIMEOUT);
        ChassisUart_ClearParser();
        return;
    }

    vx_mm_s = ChassisUart_ReadS16Be(&chassis_parser.byte[3]);
    vy_mm_s = ChassisUart_ReadS16Be(&chassis_parser.byte[5]);
    wz_mrad_s = ChassisUart_ReadS16Be(&chassis_parser.byte[7]);

    if ((chassis_parser.byte[1] != 0U) ||
        (chassis_parser.byte[2] != 0U) ||
        (vy_mm_s != 0) ||
        (vx_mm_s < -CHASSIS_UART_MAX_LINEAR_MM_S) ||
        (vx_mm_s > CHASSIS_UART_MAX_LINEAR_MM_S) ||
        (wz_mrad_s < -CHASSIS_UART_MAX_ANGULAR_MRAD_S) ||
        (wz_mrad_s > CHASSIS_UART_MAX_ANGULAR_MRAD_S))
    {
        ChassisUart_Disarm(CHASSIS_UART_STATUS_RX_ERROR);
        ChassisUart_ClearParser();
        return;
    }

    chassis_last_valid_ms = received_ms;
    if ((vx_mm_s == 0) && (wz_mrad_s == 0))
    {
        chassis_armed = 1U;
        chassis_vx = 0.0f;
        chassis_wz = 0.0f;
        chassis_status = CHASSIS_UART_STATUS_ACTIVE;
    }
    else if (chassis_armed != 0U)
    {
        chassis_vx = (float)vx_mm_s / 1000.0f;
        chassis_wz = (float)wz_mrad_s / 1000.0f;
        chassis_status = CHASSIS_UART_STATUS_ACTIVE;
    }

    ChassisUart_ClearParser();
}

static void ChassisUart_ParseByte(uint8_t byte, uint32_t timestamp,
                               uint32_t now_ms)
{
    if ((chassis_parser.length != 0U) &&
        (ChassisUart_Elapsed(timestamp,
                          chassis_parser.timestamp[chassis_parser.length - 1U]) >
         CHASSIS_UART_INTERBYTE_TIMEOUT_MS))
    {
        ChassisUart_Disarm(CHASSIS_UART_STATUS_RX_ERROR);
        ChassisUart_ClearParser();
    }

    if (chassis_parser.length == 0U)
    {
        if (byte != 0x7BU)
        {
            return;
        }
    }

    chassis_parser.byte[chassis_parser.length] = byte;
    chassis_parser.timestamp[chassis_parser.length] = timestamp;
    ++chassis_parser.length;

    if (chassis_parser.length < CHASSIS_UART_FRAME_SIZE)
    {
        return;
    }

    if ((chassis_parser.byte[10] != 0x7DU) ||
        (ChassisUart_Checksum(chassis_parser.byte) != chassis_parser.byte[9]))
    {
        ChassisUart_Disarm(CHASSIS_UART_STATUS_RX_ERROR);
        ChassisUart_ResyncWindow();
        return;
    }

    ChassisUart_AcceptFrame(now_ms);
}

void ChassisUart_Reset(void)
{
    chassis_rx_head = 0U;
    chassis_rx_tail = 0U;
    chassis_rx_error = 0U;
    ChassisUart_ClearParser();
    chassis_armed = 0U;
    chassis_status = CHASSIS_UART_STATUS_WAIT_ZERO;
    chassis_physical_allowed_previous = 1U;
    chassis_last_valid_ms = 0U;
    chassis_vx = 0.0f;
    chassis_wz = 0.0f;
}

void ChassisUart_PushByte(uint8_t byte, uint32_t now_ms)
{
    uint8_t head;
    uint8_t next;

    head = chassis_rx_head;
    next = (uint8_t)((head + 1U) & CHASSIS_UART_RING_MASK);
    if (next == chassis_rx_tail)
    {
        chassis_rx_error = 1U;
        return;
    }

    chassis_rx_byte[head] = byte;
    chassis_rx_timestamp[head] = now_ms;
    chassis_rx_head = next;
}

void ChassisUart_RxError(void)
{
    chassis_rx_error = 1U;
}

uint8_t ChassisUart_Step(uint32_t now_ms, uint8_t physical_allowed,
                      float *vx, float *wz)
{
    uint8_t tail;
    uint8_t byte;
    uint32_t timestamp;

    if (vx != 0)
    {
        *vx = 0.0f;
    }
    if (wz != 0)
    {
        *wz = 0.0f;
    }

    if (physical_allowed == 0U)
    {
        ChassisUart_FlushRing();
        ChassisUart_ClearParser();
        chassis_rx_error = 0U;
        ChassisUart_Disarm(CHASSIS_UART_STATUS_PHYSICAL_DISABLED);
        chassis_physical_allowed_previous = 0U;
        return 0U;
    }

    if (chassis_physical_allowed_previous == 0U)
    {
        ChassisUart_FlushRing();
        ChassisUart_ClearParser();
        chassis_rx_error = 0U;
        ChassisUart_Disarm(CHASSIS_UART_STATUS_WAIT_ZERO);
        chassis_physical_allowed_previous = 1U;
        return 0U;
    }

    if (chassis_rx_error != 0U)
    {
        chassis_rx_error = 0U;
        ChassisUart_FlushRing();
        ChassisUart_ClearParser();
        ChassisUart_Disarm(CHASSIS_UART_STATUS_RX_ERROR);
        return 0U;
    }

    while (chassis_rx_tail != chassis_rx_head)
    {
        tail = chassis_rx_tail;
        byte = chassis_rx_byte[tail];
        timestamp = chassis_rx_timestamp[tail];
        chassis_rx_tail = (uint8_t)((tail + 1U) & CHASSIS_UART_RING_MASK);
        ChassisUart_ParseByte(byte, timestamp, now_ms);
    }

    if ((chassis_parser.length != 0U) &&
        (ChassisUart_Elapsed(now_ms,
                          chassis_parser.timestamp[chassis_parser.length - 1U]) >
         CHASSIS_UART_INTERBYTE_TIMEOUT_MS))
    {
        ChassisUart_ClearParser();
        ChassisUart_Disarm(CHASSIS_UART_STATUS_RX_ERROR);
    }

    if ((chassis_armed != 0U) &&
        (ChassisUart_Elapsed(now_ms, chassis_last_valid_ms) >=
         CHASSIS_UART_COMMAND_TIMEOUT_MS))
    {
        ChassisUart_Disarm(CHASSIS_UART_STATUS_TIMEOUT);
    }

    if (chassis_armed == 0U)
    {
        return 0U;
    }

    if (vx != 0)
    {
        *vx = chassis_vx;
    }
    if (wz != 0)
    {
        *wz = chassis_wz;
    }
    return 1U;
}

uint8_t ChassisUart_Status(void)
{
    return chassis_status;
}
