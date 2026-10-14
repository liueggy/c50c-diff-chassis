"""Host tests for the UART3 differential-drive command parser.

Run directly with Python. GCC is required on PATH; no STM32 headers or
hardware are used.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "chassis_uart_control.h"

static void make_frame(uint8_t *frame, int16_t vx, int16_t vy, int16_t wz)
{
    uint16_t value;
    uint8_t checksum = 0U;
    unsigned int index;

    frame[0] = 0x7BU;
    frame[1] = 0U;
    frame[2] = 0U;
    value = (uint16_t)vx;
    frame[3] = (uint8_t)(value >> 8);
    frame[4] = (uint8_t)value;
    value = (uint16_t)vy;
    frame[5] = (uint8_t)(value >> 8);
    frame[6] = (uint8_t)value;
    value = (uint16_t)wz;
    frame[7] = (uint8_t)(value >> 8);
    frame[8] = (uint8_t)value;
    for (index = 0U; index < 9U; ++index)
    {
        checksum ^= frame[index];
    }
    frame[9] = checksum;
    frame[10] = 0x7DU;
}

static void repair_checksum(uint8_t *frame)
{
    uint8_t checksum = 0U;
    unsigned int index;
    for (index = 0U; index < 9U; ++index)
    {
        checksum ^= frame[index];
    }
    frame[9] = checksum;
}

static void push_bytes(const uint8_t *data, unsigned int length,
                       uint32_t first_ms, uint32_t spacing_ms)
{
    unsigned int index;
    for (index = 0U; index < length; ++index)
    {
        ChassisUart_PushByte(data[index], first_ms + index * spacing_ms);
    }
}

static uint8_t step(uint32_t now_ms, uint8_t allowed,
                    float expected_vx, float expected_wz)
{
    float vx = 99.0f;
    float wz = 99.0f;
    uint8_t running = ChassisUart_Step(now_ms, allowed, &vx, &wz);
    assert(fabsf(vx - expected_vx) < 0.00001f);
    assert(fabsf(wz - expected_wz) < 0.00001f);
    return running;
}

static void test_fragmented_and_multiple_frames(void)
{
    uint8_t zero[CHASSIS_UART_FRAME_SIZE];
    uint8_t command[CHASSIS_UART_FRAME_SIZE];

    ChassisUart_Reset();
    make_frame(zero, 0, 0, 0);
    make_frame(command, 125, 0, -500);
    push_bytes(zero, 5U, 10U, 1U);
    assert(step(20U, 1U, 0.0f, 0.0f) == 0U);
    push_bytes(zero + 5, 6U, 21U, 1U);
    assert(step(30U, 1U, 0.0f, 0.0f) == 1U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_ACTIVE);

    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 31U, 1U);
    make_frame(zero, 0, 0, 0);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 42U, 1U);
    assert(step(60U, 1U, 0.0f, 0.0f) == 1U);

    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 61U, 1U);
    assert(step(75U, 1U, 0.125f, -0.5f) == 1U);
}

static void test_garbage_embedded_header_and_resync(void)
{
    uint8_t zero[CHASSIS_UART_FRAME_SIZE];
    uint8_t command[CHASSIS_UART_FRAME_SIZE];
    uint8_t broken[CHASSIS_UART_FRAME_SIZE - 1U];
    uint8_t garbage[] = {0x00U, 0x7DU, 0x55U, 0xAAU};

    ChassisUart_Reset();
    make_frame(zero, 0, 0, 0);
    push_bytes(garbage, sizeof(garbage), 0U, 1U);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 4U, 1U);
    assert(step(20U, 1U, 0.0f, 0.0f) == 1U);

    /* 0x7B is legitimate payload data (123 mm/s). */
    make_frame(command, 123, 0, 123);
    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 21U, 1U);
    assert(step(35U, 1U, 0.123f, 0.123f) == 1U);

    /* Drop one byte. The next frame header must become the resync point. */
    make_frame(command, 200, 0, 300);
    memcpy(broken, command, 4U);
    memcpy(broken + 4U, command + 5U, CHASSIS_UART_FRAME_SIZE - 5U);
    push_bytes(broken, sizeof(broken), 40U, 1U);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 50U, 1U);
    assert(step(70U, 1U, 0.0f, 0.0f) == 1U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_ACTIVE);
}

static void test_bad_checksum_and_tail_require_zero(void)
{
    uint8_t zero[CHASSIS_UART_FRAME_SIZE];
    uint8_t command[CHASSIS_UART_FRAME_SIZE];

    ChassisUart_Reset();
    make_frame(zero, 0, 0, 0);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 0U, 1U);
    assert(step(20U, 1U, 0.0f, 0.0f) == 1U);

    make_frame(command, 100, 0, 200);
    command[9] ^= 0x01U;
    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 21U, 1U);
    assert(step(40U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_RX_ERROR);
    make_frame(command, 100, 0, 200);
    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 41U, 1U);
    assert(step(60U, 1U, 0.0f, 0.0f) == 0U);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 61U, 1U);
    assert(step(80U, 1U, 0.0f, 0.0f) == 1U);

    make_frame(command, 100, 0, 200);
    command[10] = 0x00U;
    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 81U, 1U);
    assert(step(100U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_RX_ERROR);
}

static void test_field_and_range_rejection(void)
{
    uint8_t frame[CHASSIS_UART_FRAME_SIZE];
    uint8_t zero[CHASSIS_UART_FRAME_SIZE];

    make_frame(zero, 0, 0, 0);

    ChassisUart_Reset();
    make_frame(frame, 701, 0, 0);
    push_bytes(frame, CHASSIS_UART_FRAME_SIZE, 0U, 1U);
    assert(step(20U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_RX_ERROR);

    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 21U, 1U);
    assert(step(40U, 1U, 0.0f, 0.0f) == 1U);
    make_frame(frame, 0, 1, 0);
    push_bytes(frame, CHASSIS_UART_FRAME_SIZE, 41U, 1U);
    assert(step(60U, 1U, 0.0f, 0.0f) == 0U);

    make_frame(frame, 0, 0, 1001);
    push_bytes(frame, CHASSIS_UART_FRAME_SIZE, 61U, 1U);
    assert(step(80U, 1U, 0.0f, 0.0f) == 0U);

    make_frame(frame, 0, 0, 0);
    frame[1] = 1U;
    repair_checksum(frame);
    push_bytes(frame, CHASSIS_UART_FRAME_SIZE, 81U, 1U);
    assert(step(100U, 1U, 0.0f, 0.0f) == 0U);

    make_frame(frame, 0, 0, 0);
    frame[2] = 1U;
    repair_checksum(frame);
    push_bytes(frame, CHASSIS_UART_FRAME_SIZE, 101U, 1U);
    assert(step(120U, 1U, 0.0f, 0.0f) == 0U);
}

static void test_interbyte_timeout_and_tick_wrap(void)
{
    uint8_t zero[CHASSIS_UART_FRAME_SIZE];

    make_frame(zero, 0, 0, 0);
    ChassisUart_Reset();
    push_bytes(zero, 5U, 0U, 1U);
    assert(step(60U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_RX_ERROR);
    push_bytes(zero + 5U, 6U, 61U, 1U);
    assert(step(70U, 1U, 0.0f, 0.0f) == 0U);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 71U, 1U);
    assert(step(90U, 1U, 0.0f, 0.0f) == 1U);

    ChassisUart_Reset();
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, UINT32_MAX - 5U, 1U);
    assert(step(10U, 1U, 0.0f, 0.0f) == 1U);
}

static void test_command_timeout_stale_data_and_rearm(void)
{
    uint8_t zero[CHASSIS_UART_FRAME_SIZE];
    uint8_t command[CHASSIS_UART_FRAME_SIZE];

    make_frame(zero, 0, 0, 0);
    make_frame(command, -300, 0, 1000);
    ChassisUart_Reset();
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 0U, 0U);
    assert(step(1U, 1U, 0.0f, 0.0f) == 1U);
    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 10U, 0U);
    assert(step(10U, 1U, -0.3f, 1.0f) == 1U);
    assert(step(509U, 1U, -0.3f, 1.0f) == 1U);
    assert(step(510U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_TIMEOUT);

    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 511U, 0U);
    assert(step(511U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_TIMEOUT);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 520U, 0U);
    assert(step(520U, 1U, 0.0f, 0.0f) == 1U);

    ChassisUart_Reset();
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 0U, 0U);
    assert(step(500U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_TIMEOUT);
}

static void test_physical_disable_flush_and_rearm(void)
{
    uint8_t zero[CHASSIS_UART_FRAME_SIZE];
    uint8_t command[CHASSIS_UART_FRAME_SIZE];

    make_frame(zero, 0, 0, 0);
    make_frame(command, 200, 0, -250);
    ChassisUart_Reset();
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 0U, 0U);
    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 1U, 0U);
    assert(step(2U, 1U, 0.2f, -0.25f) == 1U);
    assert(step(3U, 0U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_PHYSICAL_DISABLED);

    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 4U, 0U);
    assert(step(5U, 0U, 0.0f, 0.0f) == 0U);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 6U, 0U);
    assert(step(7U, 1U, 0.0f, 0.0f) == 0U); /* Recovery flush. */
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_WAIT_ZERO);

    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 8U, 0U);
    assert(step(9U, 1U, 0.0f, 0.0f) == 0U);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 10U, 0U);
    push_bytes(command, CHASSIS_UART_FRAME_SIZE, 11U, 0U);
    assert(step(12U, 1U, 0.2f, -0.25f) == 1U);
}

static void test_ring_overflow_and_hardware_error(void)
{
    uint8_t zero[CHASSIS_UART_FRAME_SIZE];
    unsigned int index;

    make_frame(zero, 0, 0, 0);
    ChassisUart_Reset();
    for (index = 0U; index < CHASSIS_UART_RX_RING_SIZE; ++index)
    {
        ChassisUart_PushByte(0x55U, index);
    }
    assert(step(200U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_RX_ERROR);
    push_bytes(zero, CHASSIS_UART_FRAME_SIZE, 201U, 0U);
    assert(step(202U, 1U, 0.0f, 0.0f) == 1U);

    ChassisUart_RxError();
    assert(step(203U, 1U, 0.0f, 0.0f) == 0U);
    assert(ChassisUart_Status() == CHASSIS_UART_STATUS_RX_ERROR);
}

int main(void)
{
    test_fragmented_and_multiple_frames();
    test_garbage_embedded_header_and_resync();
    test_bad_checksum_and_tail_require_zero();
    test_field_and_range_rejection();
    test_interbyte_timeout_and_tick_wrap();
    test_command_timeout_stale_data_and_rearm();
    test_physical_disable_flush_and_rearm();
    test_ring_overflow_and_hardware_error();
    return 0;
}
'''


def main():
    gcc = shutil.which("gcc")
    if not gcc:
        raise SystemExit("GCC is required on PATH for host tests.")

    with tempfile.TemporaryDirectory(prefix="chassis-uart-tests-") as temporary:
        temporary_path = Path(temporary)
        harness = temporary_path / "harness.c"
        binary = temporary_path / "chassis_uart_test.exe"
        harness.write_text(HARNESS, encoding="ascii")
        subprocess.run(
            [
                gcc,
                "-std=c99",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I" + str(ROOT / "BALANCE/Inc"),
                str(ROOT / "BALANCE/chassis_uart_control.c"),
                str(harness),
                "-o",
                str(binary),
            ],
            check=True,
        )
        subprocess.run([str(binary)], check=True)
        print("PASS chassis_uart_control")


if __name__ == "__main__":
    main()
