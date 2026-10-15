"""Exercise the real two-input parser and exclusive-control arbitration."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HARNESS = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "chassis_uart_control.h"
static void send(int source,int mode,int vx,int wz,unsigned now) {
    uint8_t f[11]={0x7b,0,0,0,0,0,0,0,0,0,0x7d}; f[1]=mode;
    f[3]=(uint16_t)vx>>8; f[4]=vx; f[7]=(uint16_t)wz>>8; f[8]=wz;
    for(int i=0;i<9;i++) f[9]^=f[i];
    for(int i=0;i<11;i++) {
        if(source==3) ChassisUart_PushByte(f[i],now);
        else ChassisBluetooth_PushByte(f[i],now);
    }
}
int main(void) {
    float vx,wz;
    ChassisUart_Reset();
    send(4,0,200,0,10); assert(!ChassisControl_Step(10,1,&vx,&wz));
    assert(ChassisControl_Source()==0); /* nonzero cannot claim */
    send(4,0,0,0,20); assert(ChassisControl_Step(20,1,&vx,&wz));
    assert(ChassisControl_Source()==4 && vx==0);
    send(4,0,-200,500,30); assert(ChassisControl_Step(30,1,&vx,&wz));
    assert(fabsf(vx+0.2f)<1e-5f && fabsf(wz-0.5f)<1e-5f);
    send(3,0,0,0,31); send(3,0,700,0,31); ChassisUart_RxError();
    assert(ChassisControl_Step(31,1,&vx,&wz) && vx<0 && ChassisControl_Source()==4);
    send(3,0x10,0,0,32); assert(ChassisControl_Step(32,1,&vx,&wz));
    assert(ChassisControl_Source()==4); /* nonowner release ignored */
    assert(!ChassisControl_Step(530,1,&vx,&wz));
    assert(vx==0 && wz==0 && ChassisControl_Source()==0);
    assert(ChassisControl_Status()==CHASSIS_UART_STATUS_TIMEOUT);
    ChassisControl_Step(531,1,&vx,&wz); /* flush recovery boundary */
    send(3,0,400,0,532); assert(!ChassisControl_Step(532,1,&vx,&wz));
    send(3,0,0,0,540); assert(ChassisControl_Step(540,1,&vx,&wz));
    assert(ChassisControl_Source()==3);
    send(3,0,100,0,550); assert(ChassisControl_Step(550,1,&vx,&wz));
    send(3,0x10,0,0,560); assert(!ChassisControl_Step(560,1,&vx,&wz));
    assert(vx==0 && ChassisControl_Source()==0);
    ChassisControl_Step(561,1,&vx,&wz);
    send(4,0,0,0,570); assert(ChassisControl_Step(570,1,&vx,&wz));
    assert(ChassisControl_Source()==4);
    send(4,0,100,0,580); assert(ChassisControl_Step(580,1,&vx,&wz));
    assert(!ChassisControl_Step(581,0,&vx,&wz));
    send(4,0,100,0,582); assert(!ChassisControl_Step(582,1,&vx,&wz));
    send(4,0,100,0,583); assert(!ChassisControl_Step(583,1,&vx,&wz));
    send(4,0,0,0,590); assert(ChassisControl_Step(590,1,&vx,&wz));
    ChassisBluetooth_RxError(); assert(!ChassisControl_Step(591,1,&vx,&wz));
    assert(ChassisControl_Status()==CHASSIS_UART_STATUS_RX_ERROR && vx==0);
    ChassisUart_Reset(); send(4,0,0,0,600); send(3,0,0,0,600);
    assert(ChassisControl_Step(600,1,&vx,&wz) && ChassisControl_Source()==3);
    send(3,0x10,100,0,610); assert(!ChassisControl_Step(610,1,&vx,&wz));
    assert(ChassisControl_Status()==CHASSIS_UART_STATUS_RX_ERROR);
    /* A header and payload from different links must never form a command. */
    ChassisUart_Reset();
    ChassisUart_PushByte(0x7b,620);
    uint8_t tail[10]={0,0,0,0,0,0,0,0,0x7b,0x7d};
    for(int i=0;i<10;i++) ChassisBluetooth_PushByte(tail[i],620);
    assert(!ChassisControl_Step(620,1,&vx,&wz) && ChassisControl_Source()==0);
    puts("PASS Bluetooth/UART3 isolation, ownership, release, timeout and physical recovery");
}
'''


if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="bluetooth-control-") as directory:
        temp = Path(directory)
        (temp / "test.c").write_text(HARNESS, encoding="ascii")
        binary = temp / "test.exe"
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Werror", "-I" + str(ROOT / "BALANCE/Inc"),
                        str(temp / "test.c"), str(ROOT / "BALANCE/chassis_uart_control.c"),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
