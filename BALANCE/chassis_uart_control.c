#include "chassis_uart_control.h"

#define CHASSIS_UART_RING_MASK (CHASSIS_UART_RX_RING_SIZE - 1U)

typedef struct
{
    uint8_t byte[CHASSIS_UART_FRAME_SIZE];
    uint32_t timestamp[CHASSIS_UART_FRAME_SIZE];
    uint8_t length;
} ChassisUartParser;

typedef struct
{
    volatile uint8_t rx_byte[CHASSIS_UART_RX_RING_SIZE];
    volatile uint32_t rx_timestamp[CHASSIS_UART_RX_RING_SIZE];
    volatile uint8_t rx_head, rx_tail, rx_error;
    ChassisUartParser parser;
    uint8_t armed, status, physical_allowed_previous, release_requested;
    uint32_t last_valid_ms;
    float vx, wz;
} ChassisUartContext;

static ChassisUartContext chassis_inputs[2];
static uint8_t chassis_owner;
static uint8_t chassis_control_status;

static uint32_t ChassisUart_Elapsed(uint32_t now_ms, uint32_t then_ms)
{
    return now_ms - then_ms;
}

static void ChassisContext_ClearParser(ChassisUartContext *ctx)
{
    ctx->parser.length = 0U;
}

static void ChassisContext_Disarm(ChassisUartContext *ctx, uint8_t status)
{
    ctx->armed = 0U;
    ctx->vx = 0.0f;
    ctx->wz = 0.0f;
    ctx->status = status;
}

static void ChassisContext_FlushRing(ChassisUartContext *ctx)
{
    ctx->rx_tail = ctx->rx_head;
}

static int16_t ChassisUart_ReadS16Be(const uint8_t *data)
{
    uint16_t value;

    value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    return (int16_t)value;
}

static void ChassisContext_ResyncWindow(ChassisUartContext *ctx)
{
    uint8_t start;
    uint8_t index;
    uint8_t remaining;

    for (start = 1U; start < ctx->parser.length; ++start)
    {
        if (ctx->parser.byte[start] == 0x7BU)
        {
            break;
        }
    }

    if (start >= ctx->parser.length)
    {
        ChassisContext_ClearParser(ctx);
        return;
    }

    remaining = (uint8_t)(ctx->parser.length - start);
    for (index = 0U; index < remaining; ++index)
    {
        ctx->parser.byte[index] = ctx->parser.byte[start + index];
        ctx->parser.timestamp[index] = ctx->parser.timestamp[start + index];
    }
    ctx->parser.length = remaining;
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

static void ChassisContext_AcceptFrame(ChassisUartContext *ctx, uint32_t now_ms)
{
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t wz_mrad_s;
    uint32_t received_ms;

    received_ms = ctx->parser.timestamp[CHASSIS_UART_FRAME_SIZE - 1U];
    if (ChassisUart_Elapsed(now_ms, received_ms) >= CHASSIS_UART_COMMAND_TIMEOUT_MS)
    {
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_TIMEOUT);
        ChassisContext_ClearParser(ctx);
        return;
    }

    vx_mm_s = ChassisUart_ReadS16Be(&ctx->parser.byte[3]);
    vy_mm_s = ChassisUart_ReadS16Be(&ctx->parser.byte[5]);
    wz_mrad_s = ChassisUart_ReadS16Be(&ctx->parser.byte[7]);

    if (((ctx->parser.byte[1] != 0U) && (ctx->parser.byte[1] != CHASSIS_COMMAND_RELEASE)) ||
        (ctx->parser.byte[2] != 0U) ||
        (vy_mm_s != 0) ||
        (vx_mm_s < -CHASSIS_UART_MAX_LINEAR_MM_S) ||
        (vx_mm_s > CHASSIS_UART_MAX_LINEAR_MM_S) ||
        (wz_mrad_s < -CHASSIS_UART_MAX_ANGULAR_MRAD_S) ||
        (wz_mrad_s > CHASSIS_UART_MAX_ANGULAR_MRAD_S))
    {
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_RX_ERROR);
        ChassisContext_ClearParser(ctx);
        return;
    }

    if (ctx->parser.byte[1] == CHASSIS_COMMAND_RELEASE)
    {
        if (vx_mm_s != 0 || wz_mrad_s != 0)
            ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_RX_ERROR);
        else
        {
            ctx->release_requested = 1U;
            ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_WAIT_ZERO);
        }
        ChassisContext_ClearParser(ctx);
        return;
    }

    ctx->last_valid_ms = received_ms;
    if ((vx_mm_s == 0) && (wz_mrad_s == 0))
    {
        ctx->armed = 1U;
        ctx->vx = 0.0f;
        ctx->wz = 0.0f;
        ctx->status = CHASSIS_UART_STATUS_ACTIVE;
    }
    else if (ctx->armed != 0U)
    {
        ctx->vx = (float)vx_mm_s / 1000.0f;
        ctx->wz = (float)wz_mrad_s / 1000.0f;
        ctx->status = CHASSIS_UART_STATUS_ACTIVE;
    }

    ChassisContext_ClearParser(ctx);
}

static void ChassisContext_ParseByte(ChassisUartContext *ctx, uint8_t byte, uint32_t timestamp,
                               uint32_t now_ms)
{
    if ((ctx->parser.length != 0U) &&
        (ChassisUart_Elapsed(timestamp,
                          ctx->parser.timestamp[ctx->parser.length - 1U]) >
         CHASSIS_UART_INTERBYTE_TIMEOUT_MS))
    {
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_RX_ERROR);
        ChassisContext_ClearParser(ctx);
    }

    if (ctx->parser.length == 0U)
    {
        if (byte != 0x7BU)
        {
            return;
        }
    }

    ctx->parser.byte[ctx->parser.length] = byte;
    ctx->parser.timestamp[ctx->parser.length] = timestamp;
    ++ctx->parser.length;

    if (ctx->parser.length < CHASSIS_UART_FRAME_SIZE)
    {
        return;
    }

    if ((ctx->parser.byte[10] != 0x7DU) ||
        (ChassisUart_Checksum(ctx->parser.byte) != ctx->parser.byte[9]))
    {
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_RX_ERROR);
        ChassisContext_ResyncWindow(ctx);
        return;
    }

    ChassisContext_AcceptFrame(ctx, now_ms);
}

static void ChassisContext_Reset(ChassisUartContext *ctx)
{
    ctx->rx_head = 0U;
    ctx->rx_tail = 0U;
    ctx->rx_error = 0U;
    ctx->release_requested = 0U;
    ChassisContext_ClearParser(ctx);
    ctx->armed = 0U;
    ctx->status = CHASSIS_UART_STATUS_WAIT_ZERO;
    ctx->physical_allowed_previous = 1U;
    ctx->last_valid_ms = 0U;
    ctx->vx = 0.0f;
    ctx->wz = 0.0f;
}

static void ChassisContext_PushByte(ChassisUartContext *ctx, uint8_t byte, uint32_t now_ms)
{
    uint8_t head;
    uint8_t next;

    head = ctx->rx_head;
    next = (uint8_t)((head + 1U) & CHASSIS_UART_RING_MASK);
    if (next == ctx->rx_tail)
    {
        ctx->rx_error = 1U;
        return;
    }

    ctx->rx_byte[head] = byte;
    ctx->rx_timestamp[head] = now_ms;
    ctx->rx_head = next;
}

static void ChassisContext_RxError(ChassisUartContext *ctx)
{
    ctx->rx_error = 1U;
}

static uint8_t ChassisContext_Step(ChassisUartContext *ctx, uint32_t now_ms, uint8_t physical_allowed,
                      float *vx, float *wz)
{
    uint8_t tail;
    uint8_t byte;
    uint32_t timestamp;

    ctx->release_requested = 0U;

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
        ChassisContext_FlushRing(ctx);
        ChassisContext_ClearParser(ctx);
        ctx->rx_error = 0U;
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_PHYSICAL_DISABLED);
        ctx->physical_allowed_previous = 0U;
        return 0U;
    }

    if (ctx->physical_allowed_previous == 0U)
    {
        ChassisContext_FlushRing(ctx);
        ChassisContext_ClearParser(ctx);
        ctx->rx_error = 0U;
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_WAIT_ZERO);
        ctx->physical_allowed_previous = 1U;
        return 0U;
    }

    if (ctx->rx_error != 0U)
    {
        ctx->rx_error = 0U;
        ChassisContext_FlushRing(ctx);
        ChassisContext_ClearParser(ctx);
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_RX_ERROR);
        return 0U;
    }

    while (ctx->rx_tail != ctx->rx_head)
    {
        tail = ctx->rx_tail;
        byte = ctx->rx_byte[tail];
        timestamp = ctx->rx_timestamp[tail];
        ctx->rx_tail = (uint8_t)((tail + 1U) & CHASSIS_UART_RING_MASK);
        ChassisContext_ParseByte(ctx, byte, timestamp, now_ms);
    }

    if ((ctx->parser.length != 0U) &&
        (ChassisUart_Elapsed(now_ms,
                          ctx->parser.timestamp[ctx->parser.length - 1U]) >
         CHASSIS_UART_INTERBYTE_TIMEOUT_MS))
    {
        ChassisContext_ClearParser(ctx);
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_RX_ERROR);
    }

    if ((ctx->armed != 0U) &&
        (ChassisUart_Elapsed(now_ms, ctx->last_valid_ms) >=
         CHASSIS_UART_COMMAND_TIMEOUT_MS))
    {
        ChassisContext_Disarm(ctx, CHASSIS_UART_STATUS_TIMEOUT);
    }

    if (ctx->armed == 0U)
    {
        return 0U;
    }

    if (vx != 0)
    {
        *vx = ctx->vx;
    }
    if (wz != 0)
    {
        *wz = ctx->wz;
    }
    return 1U;
}

void ChassisUart_Reset(void)
{
    ChassisContext_Reset(&chassis_inputs[0]);
    ChassisContext_Reset(&chassis_inputs[1]);
    chassis_owner = CHASSIS_SOURCE_NONE;
    chassis_control_status = CHASSIS_UART_STATUS_WAIT_ZERO;
}

void ChassisUart_PushByte(uint8_t byte, uint32_t now_ms)
{
    ChassisContext_PushByte(&chassis_inputs[0], byte, now_ms);
}
void ChassisUart_RxError(void) { ChassisContext_RxError(&chassis_inputs[0]); }
uint8_t ChassisUart_Step(uint32_t now_ms, uint8_t allowed, float *vx, float *wz)
{
    return ChassisContext_Step(&chassis_inputs[0], now_ms, allowed, vx, wz);
}
uint8_t ChassisUart_Status(void) { return chassis_inputs[0].status; }

void ChassisBluetooth_PushByte(uint8_t byte, uint32_t now_ms)
{
    ChassisContext_PushByte(&chassis_inputs[1], byte, now_ms);
}
void ChassisBluetooth_RxError(void) { ChassisContext_RxError(&chassis_inputs[1]); }

uint8_t ChassisControl_Step(uint32_t now_ms, uint8_t allowed, float *vx, float *wz)
{
    float x[2] = {0.0f, 0.0f}, z[2] = {0.0f, 0.0f};
    uint8_t active[2], index, other;
    if (vx) *vx = 0.0f;
    if (wz) *wz = 0.0f;
    if (!allowed)
    {
        ChassisContext_Step(&chassis_inputs[0], now_ms, 0, &x[0], &z[0]);
        ChassisContext_Step(&chassis_inputs[1], now_ms, 0, &x[1], &z[1]);
        chassis_owner = CHASSIS_SOURCE_NONE;
        chassis_control_status = CHASSIS_UART_STATUS_PHYSICAL_DISABLED;
        return 0;
    }
    if (chassis_owner != CHASSIS_SOURCE_NONE)
    {
        index = chassis_owner == CHASSIS_SOURCE_UART3 ? 0 : 1;
        other = 1 - index;
        ChassisContext_Step(&chassis_inputs[other], now_ms, 0, &x[other], &z[other]);
        active[index] = ChassisContext_Step(&chassis_inputs[index], now_ms, 1, &x[index], &z[index]);
        chassis_control_status = chassis_inputs[index].status;
        if (!active[index] || chassis_inputs[index].release_requested)
        {
            ChassisContext_Step(&chassis_inputs[index], now_ms, 0, &x[index], &z[index]);
            chassis_owner = CHASSIS_SOURCE_NONE;
            return 0;
        }
    }
    else
    {
        active[0] = ChassisContext_Step(&chassis_inputs[0], now_ms, 1, &x[0], &z[0]);
        active[1] = ChassisContext_Step(&chassis_inputs[1], now_ms, 1, &x[1], &z[1]);
        /* Only a zero frame can acquire control. UART3 wins a simultaneous tie. */
        for (index = 0; index < 2; ++index)
        {
            if (chassis_inputs[index].release_requested)
            {
                ChassisContext_Step(&chassis_inputs[index], now_ms, 0, &x[index], &z[index]);
                active[index] = 0;
            }
            if (active[index] && x[index] == 0.0f && z[index] == 0.0f) break;
        }
        if (index == 2)
        {
            if (chassis_inputs[0].status == CHASSIS_UART_STATUS_RX_ERROR || chassis_inputs[1].status == CHASSIS_UART_STATUS_RX_ERROR)
                chassis_control_status = CHASSIS_UART_STATUS_RX_ERROR;
            else if (chassis_inputs[0].status == CHASSIS_UART_STATUS_TIMEOUT || chassis_inputs[1].status == CHASSIS_UART_STATUS_TIMEOUT)
                chassis_control_status = CHASSIS_UART_STATUS_TIMEOUT;
            return 0;
        }
        chassis_owner = index == 0 ? CHASSIS_SOURCE_UART3 : CHASSIS_SOURCE_BLUETOOTH;
        chassis_control_status = CHASSIS_UART_STATUS_ACTIVE;
        other = 1 - index;
        ChassisContext_Step(&chassis_inputs[other], now_ms, 0, &x[other], &z[other]);
    }
    if (vx) *vx = x[index];
    if (wz) *wz = z[index];
    return 1;
}
uint8_t ChassisControl_Status(void) { return chassis_control_status; }
uint8_t ChassisControl_Source(void) { return chassis_owner; }
