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
    /* Display activity is independent of motion ownership and physical enable. */
    ChassisUart_Reset(); assert(!ChassisUart_LinkOnline(0));
    send(4,0,0,0,1000); assert(ChassisControl_Step(1000,1,&vx,&wz));
    assert(!ChassisUart_LinkOnline(1000)); /* Bluetooth cannot mark ROS online. */
    send(3,0,0,0,1010); assert(ChassisControl_Step(1010,1,&vx,&wz));
    assert(ChassisUart_LinkOnline(1010) && ChassisControl_Source()==4);
    send(3,0,200,0,1400); ChassisControl_Step(1400,1,&vx,&wz);
    assert(ChassisUart_LinkOnline(2399) && !ChassisUart_LinkOnline(2400));
    assert(ChassisControl_Source()==4 && vx==0); /* Observer never drives. */
    send(3,0,0,0,2500); assert(!ChassisControl_Step(2500,0,&vx,&wz));
    assert(ChassisUart_LinkOnline(2500) && ChassisControl_Source()==0);
    send(3,0,701,0,2600); ChassisControl_Step(2600,0,&vx,&wz);
    assert(!ChassisUart_LinkOnline(3500)); /* Invalid range cannot refresh. */
    ChassisUart_Reset();
    send(3,0,0,0,0xfffffff0U); ChassisControl_Step(0xfffffff0U,1,&vx,&wz);
    assert(ChassisUart_LinkOnline(0x10U));
    assert(!ChassisUart_LinkOnline(0x3d8U)); /* Tick wrap, exactly 1000 ms. */
    ChassisUart_Reset(); send(3,0,0,0,4000); ChassisControl_Step(4000,1,&vx,&wz);
    assert(!ChassisControl_Step(4500,1,&vx,&wz));
    assert(ChassisUart_LinkOnline(4500)); /* Display grace does not delay stop. */
    puts("PASS Bluetooth/UART3 isolation, ownership, release, timeout and physical recovery");
}
'''


WEB_HARNESS = r"""
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync(process.argv[2], 'utf8');
const code = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {id, hidden: id === 'stick', value: '', style: {},
    dataset: {}, disabled: false, textContent: '', tagName:'BUTTON',
    classList:{add(){}, remove(){}, toggle(){}}, listeners:{},
    setAttribute(k,v){this[k]=v;}, addEventListener(k,v){this.listeners[k]=v;},
    setPointerCapture(){}, getBoundingClientRect(){return {left:0,top:0,width:202,height:202};}});
  return elements.get(id);
}
const buttons = ['forward','left','stop','right','back'].map(element);
buttons.forEach((b,i)=>b.dataset=i===2?{stop:'1'}:{jx:String([-0,-1,0,1,0][i]),jy:String([-1,0,0,0,1][i])});
const events = {}, winEvents = {}, timers = [];
let clock = 1000;
class MockSocket {
  static instances=[];
  constructor(url){this.url=url;this.readyState=1;this.bufferedAmount=0;this.sent=[];MockSocket.instances.push(this);}
  send(s){this.sent.push(JSON.parse(s));} close(){this.readyState=3;if(this.onclose)this.onclose();}
}
const context = {console,URL,Uint8Array,DataView,Promise,Number,Math,JSON,Map,
  navigator:{}, location:{protocol:'http:',hostname:'192.168.31.50',host:'192.168.31.50:8765'},
  performance:{now:()=>clock}, WebSocket:MockSocket,
  document:{body:{classList:{toggle(){}}},hidden:false,getElementById:element,addEventListener:(k,v)=>events[k]=v,
    querySelectorAll:s=>s==='.dbtn.active'?[]:s.includes('.stop')?buttons.filter(b=>!b.dataset.stop):buttons},
  window:{addEventListener:(k,v)=>winEvents[k]=v},
  setInterval:fn=>timers.push(fn), setTimeout:()=>1,clearTimeout(){},requestAnimationFrame(){},
  matchMedia:()=>({matches:false})};
vm.createContext(context);vm.runInContext(code,context);
async function run(){
  const c=context;
  assert.equal(element('stick').hidden,true,'direction pad is the default');
  assert.ok(!html.includes('加速度')&&!html.includes('角速度'),'IMU panels removed');
  assert.ok(html.indexOf('id="parameter-heading"')<html.indexOf('id="control-heading"'),'parameters precede controls');
  assert.ok(!html.includes('order:-1'),'mobile preserves parameter-first order');
  const ids=new Set([...html.matchAll(/id="([^"]+)"/g)].map(m=>m[1]));
  for(const match of code.matchAll(/\$\('([^']+)'\)/g))assert.ok(ids.has(match[1]),'missing DOM '+match[1]);
  c.renderMetrics(1250);
  assert.equal(c.MAXV,150); assert.equal(c.MAXW,400);
  const f=c.buildFrame(-150,400,false);assert.equal(f.length,11);assert.equal(c.s16(f,3),-150);assert.equal(c.s16(f,7),400);assert.equal(c.xorN(f,9),f[9]);
  c.mode='nx';element('nxAddress').value='192.168.31.50:8765';element('nxToken').value='secret';c.connectNx();
  let ws=MockSocket.instances.at(-1);ws.onopen();assert.deepEqual(ws.sent.at(-1),{type:'auth',token:'secret'});
  ws.onmessage({data:JSON.stringify({type:'hello'})});assert.equal(ws.sent.at(-1).type,'claim');assert.equal(c.canDrive(),false);
  ws.onmessage({data:JSON.stringify({type:'claimed',lease:'lease-1'})});assert.equal(c.canDrive(),false,'claim alone cannot enable movement');
  const telem={type:'telemetry',lease:'lease-2',ready:true,enabled:true,owner:3,state:1,status:33,voltage_v:24,left_mps:0,right_mps:0,acc:[1,2,3],gyro:[4,5,6],fresh:true};
  ws.onmessage({data:JSON.stringify(telem)});assert.equal(c.canDrive(),true);
  events.keydown({target:{tagName:'BODY'},code:'ArrowUp',preventDefault(){},repeat:false});c.tickSend();
  let command=ws.sent.at(-1);assert.equal(command.vx,150);assert.equal(command.wz,0);assert.equal(command.lease,'lease-2');let previousSeq=command.seq;
  winEvents.blur();assert.equal(c.jy,0);assert.equal(ws.sent.at(-1).vx,0);assert.ok(ws.sent.at(-1).seq>previousSeq);
  events.keydown({target:{tagName:'BODY'},code:'ArrowUp',preventDefault(){},repeat:true});c.tickSend();assert.equal(ws.sent.at(-1).vx,0,'key repeat cannot resume a cleared gesture');
  events.keydown({target:{tagName:'INPUT'},code:'ArrowUp',preventDefault(){},repeat:false});assert.equal(c.jy,0,'editing address cannot drive');
  c.jy=-1;clock+=700;c.tickSend();assert.equal(ws.sent.at(-1).vx,0,'stale feedback stops');
  ws.onmessage({data:JSON.stringify(telem)});assert.equal(c.jy,0,'fresh feedback does not resume old gesture');
  c.jy=-1;element('btnControlMode').onclick();assert.equal(c.jy,0);assert.equal(element('stick').hidden,false);assert.equal(element('dpad').hidden,true);
  await c.releaseControl();assert.equal(ws.sent.at(-1).type,'release');assert.equal(c.canDrive(),false);let n=ws.sent.length;c.tickSend();assert.equal(ws.sent.length,n,'released page sends no speed heartbeats');
  c.rearm();assert.equal(ws.sent.at(-1).type,'claim');assert.equal(c.paused,true,'wait for claim acknowledgment');
  c.location.protocol='https:';assert.throws(()=>c.socketAddress('ws://192.168.31.50:8765'),/安全地址/);assert.equal(c.socketAddress('https://nx.example:443/ws'),'wss://nx.example/ws');c.location.protocol='http:';
  c.mode='ble';c.connectionEpoch++;c.connected=true;c.paused=false;c.claimed=true;c.owner=4;c.ownerStatus=1;c.statusByte=33;c.enabled=c.ready=c.fresh=c.hasTelemetry=true;c.lastRxAt=c.ownerAt=clock;
  assert.equal(c.canDrive(),true);c.owner=3;assert.equal(c.canDrive(),false,'Bluetooth may not drive UART owner');c.owner=4;
  let writes=[],unblock;c.writeChar={properties:{writeWithoutResponse:true},writeValueWithoutResponse(buf){writes.push(new Uint8Array(buf));if(writes.length===1)return new Promise(r=>unblock=r);return Promise.resolve();}};
  c.bleWrite(c.buildFrame(80,0,false));c.bleWrite(c.buildFrame(100,0,false));c.stopHeld();unblock();await c.bleDrain;
  assert.equal(writes.length,2);assert.equal(c.s16(writes[1],3),0,'stop replaces queued movement');
  const t=new Uint8Array(22);t[0]=0x5a;t[1]=33;t[6]=0x5d;t[7]=0xc0;t[20]=c.xorN(t,20);t[21]=0xa5;
  const prefix=new Uint8Array([99,88,...t.slice(0,7)]);c.rxBuf=new Uint8Array(0);c.onNotify({target:{value:new DataView(prefix.buffer)}});assert.equal(c.rxBuf.length,7);
  const rest=t.slice(7);c.onNotify({target:{value:new DataView(rest.buffer)}});assert.equal(c.metricTarget.v,24);assert.equal(c.rxBuf.length,0,'split notification reassembles');
  await c.releaseControl();assert.equal(writes.at(-1)[1],16,'Bluetooth release follows zero');
  console.log('PASS web dual transport, limits, claim/lease, stop, stale feedback, bounded BLE queue and telemetry');
}
run().catch(e=>{console.error(e);process.exitCode=1;});
"""


def check_control_page():
    with tempfile.TemporaryDirectory(prefix="chassis-control-web-") as directory:
        harness = Path(directory) / "web.cjs"
        harness.write_text(WEB_HARNESS, encoding="utf-8")
        subprocess.run(["node", str(harness), str(ROOT / "tools/bluetooth_control.html")], check=True)


if __name__ == "__main__":
    check_control_page()
    with tempfile.TemporaryDirectory(prefix="bluetooth-control-") as directory:
        temp = Path(directory)
        (temp / "test.c").write_text(HARNESS, encoding="ascii")
        binary = temp / "test.exe"
        subprocess.run(["gcc", "-std=c99", "-Wall", "-Werror", "-I" + str(ROOT / "BALANCE/Inc"),
                        str(temp / "test.c"), str(ROOT / "BALANCE/chassis_uart_control.c"),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
