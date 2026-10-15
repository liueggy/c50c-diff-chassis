"""Compile the actual UART4 TX implementation against mocked STM registers."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "HARDWARE/uartx.c").read_bytes().decode("gb18030")
start = source.index("#if CHASSIS_ENABLED\n/* Single telemetry-task", 0) if "#if CHASSIS_ENABLED\n/* Single telemetry-task" in source else source.index("#if CHASSIS_ENABLED\r\n/* Single telemetry-task")
end = source.index("#endif", start) + len("#endif")
implementation = source[start:end]
harness = r'''
#include <stdint.h>
#include <assert.h>
#define CHASSIS_ENABLED 1
#define USART_SR_TXE 128
#define USART_CR1_TXEIE 128
#define USART_IT_TXE 128
#define ENABLE 1
#define DISABLE 0
struct Registers { uint32_t SR, DR, CR1; } registers;
#define UART4 (&registers)
void USART_ITConfig(struct Registers *port, int interrupt, int enabled)
{
    (void)interrupt;
    if (enabled) port->CR1 |= USART_CR1_TXEIE;
    else port->CR1 &= ~USART_CR1_TXEIE;
}
''' + implementation + r'''
int main(void)
{
    uint8_t data[28], replacement[28];
    int i, length;
    for (i=0;i<28;++i) { data[i]=(uint8_t)(i+1); replacement[i]=99; }
    assert(!ChassisBluetooth_SendTelemetry(0, 22));
    assert(!ChassisBluetooth_SendTelemetry(data, 0));
    assert(!ChassisBluetooth_SendTelemetry(data, 29));
    for (length=22;length<=28;length+=6) {
        assert(ChassisBluetooth_SendTelemetry(data, (uint8_t)length));
        assert(!ChassisBluetooth_SendTelemetry(replacement, (uint8_t)length));
        UART4->DR=200; UART4->SR=0;
        ChassisBluetooth_TxIRQ(); assert(UART4->DR==200);
        UART4->SR=USART_SR_TXE;
        USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
        ChassisBluetooth_TxIRQ(); assert(UART4->DR==200);
        USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
        for(i=0;i<length;++i) {
            ChassisBluetooth_TxIRQ(); assert(UART4->DR==data[i]);
        }
        assert(!(UART4->CR1 & USART_CR1_TXEIE));
        assert(bluetooth_tx_length==0);
        UART4->DR=201; ChassisBluetooth_TxIRQ(); assert(UART4->DR==201);
    }
    return 0;
}
'''
with tempfile.TemporaryDirectory() as temporary:
    directory = Path(temporary)
    c_file = directory / "bluetooth_tx.c"
    exe_file = directory / "bluetooth_tx.exe"
    c_file.write_text(harness, encoding="ascii")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(c_file), "-o", str(exe_file)], check=True)
    subprocess.run([str(exe_file)], check=True)
print("UART4 TX: complete 22/28-byte frames, busy isolation and IRQ guards passed")
