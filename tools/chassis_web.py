#!/usr/bin/env python3
"""Chassis serial dashboard (host front-end).

- Sends the 11-byte motion command frame to the board at 20 Hz (zero-arm first,
  zero on stop).
- Parses the 22-byte chassis telemetry frame (header 0x5A / tail 0xA5) and shows
  it in a small web page you open in a browser.

Telemetry frame (chassis -> host):
    [0]  0x5A                 header
    [1]  status  (bit0 ARMED, bit1 TIMEOUT, bit2 RX_ERR, bit3 WAIT/PHYS,
                  bit4 STOP, bit5 ENABLED, bit6 LOW_V, bit7 SELF_ERR)
    [2..3]  left  actual  mm/s (s16 BE)
    [4..5]  right actual  mm/s (s16 BE)
    [6..7]  voltage mV     (u16 BE)
    [8..9]  accel X   [10..11] accel Y   [12..13] accel Z   (s16)
    [14..15] gyro X   [16..17] gyro Y   [18..19] gyro Z    (s16)
    [20] XOR of bytes[0..19]
    [21] 0xA5                 tail

Usage:
    python tools/chassis_web.py --port COM3
    python tools/chassis_web.py --sim        # no serial, fake data (page preview)

Requires pyserial:  pip install pyserial
"""

import argparse
import json
import math
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
MAX_LINEAR = 700
MAX_ANGULAR = 1000

TELEM_HEADER = 0x5A
TELEM_TAIL = 0xA5
TELEM_SIZE = 22
OWNER_HEADER = 0x5B
OWNER_TAIL = 0xA6
OWNER_SIZE = 6
COMMAND_LEASE_SECONDS = 0.5
TELEMETRY_STALE_SECONDS = 0.5
WRITER_PERIOD_SECONDS = 1.0 / 20.0
SHUTDOWN_ZERO_FRAMES = 3

STATUS_ARMED = 1 << 0
STATUS_TIMEOUT = 1 << 1
STATUS_RX_ERROR = 1 << 2
STATUS_WAIT = 1 << 3
STATUS_STOP = 1 << 4
STATUS_ENABLED = 1 << 5
STATUS_LOW_VOLTAGE = 1 << 6
STATUS_SELF_ERROR = 1 << 7
STATUS_CONTROL_FAULTS = (STATUS_TIMEOUT | STATUS_RX_ERROR | STATUS_WAIT |
                         STATUS_LOW_VOLTAGE | STATUS_SELF_ERROR)


def build_command(vx, wz):
    body = bytes([FRAME_HEADER, 0x00, 0x00])
    body += struct.pack(">h", int(round(vx)))
    body += bytes([0x00, 0x00])
    body += struct.pack(">h", int(round(wz)))
    cks = 0
    for b in body:
        cks ^= b
    return body + bytes([cks, FRAME_TAIL])


class State:
    def __init__(self, clock=time.monotonic, sim=False, expected_source=None):
        self.lock = threading.Lock()
        self.clock = clock
        self.sim = sim
        self.expected_source = expected_source
        self.owner = 0
        self.owner_status = None
        self.last_owner_at = None
        self.vx = 0.0
        self.wz = 0.0
        self.command_deadline = 0.0
        self.status = 0
        self.left_act = 0
        self.right_act = 0
        self.voltage_mv = 0
        self.ax = self.ay = self.az = 0
        self.gx = self.gy = self.gz = 0
        self.rx_ok = 0
        self.last_telemetry_at = None
        self.serial_fault = False
        self.control_ready = False
        self.operation_blocked = False

    def _clear_command_locked(self):
        self.vx = 0.0
        self.wz = 0.0
        self.command_deadline = 0.0

    def clear_command(self, release=False):
        with self.lock:
            self._clear_command_locked()
            self.operation_blocked = not release

    def set_command(self, vx, wz):
        with self.lock:
            self._expire_command_locked(self.clock())
            if vx == 0.0 and wz == 0.0:
                self._clear_command_locked()
                self.operation_blocked = False
                return True
            if not self.control_ready or self.operation_blocked:
                self._clear_command_locked()
                self.operation_blocked = True
                return False
            self.vx = vx
            self.wz = wz
            self.command_deadline = self.clock() + COMMAND_LEASE_SECONDS
            return True

    def serial_failed(self):
        with self.lock:
            self.serial_fault = True
            self.control_ready = False
            self.operation_blocked = True
            self._clear_command_locked()

    def apply_telemetry(self, frame):
        now = self.clock()
        with self.lock:
            self.status = frame[1]
            self.left_act = struct.unpack(">h", frame[2:4])[0]
            self.right_act = struct.unpack(">h", frame[4:6])[0]
            self.voltage_mv = struct.unpack(">H", frame[6:8])[0]
            self.ax, self.ay, self.az = struct.unpack(">hhh", frame[8:14])
            self.gx, self.gy, self.gz = struct.unpack(">hhh", frame[14:20])
            self.rx_ok += 1
            self.last_telemetry_at = now
            self.serial_fault = False
            if ((not self.status & STATUS_ENABLED or self.status & STATUS_CONTROL_FAULTS)
                    and (self.command_deadline > 0.0 or self.vx != 0.0 or self.wz != 0.0)):
                self.operation_blocked = True
            self._update_ready_locked(now)

    def apply_owner(self, frame):
        with self.lock:
            self.owner = frame[2]
            self.owner_status = frame[3]
            self.last_owner_at = self.clock()
            self._update_ready_locked(self.last_owner_at)

    def _owner_ready_locked(self, now):
        return (self.expected_source is None or
                (self.owner == self.expected_source
                 and self.owner_status == 1
                 and self.last_owner_at is not None
                 and now - self.last_owner_at < TELEMETRY_STALE_SECONDS))

    def _update_ready_locked(self, now):
        self.control_ready = (
            bool(self.status & STATUS_ARMED)
            and bool(self.status & STATUS_ENABLED)
            and not bool(self.status & STATUS_CONTROL_FAULTS)
            and self._owner_ready_locked(now)
            and not self.serial_fault
            and self.last_telemetry_at is not None
            and now - self.last_telemetry_at < TELEMETRY_STALE_SECONDS
        )
        if not self.control_ready:
            if self.command_deadline > 0.0 or self.vx != 0.0 or self.wz != 0.0:
                self.operation_blocked = True
            self._clear_command_locked()

    def _expire_command_locked(self, now):
        had_active_command = (self.command_deadline > 0.0
                              or self.vx != 0.0 or self.wz != 0.0)
        telemetry_stale = (
            not self.sim
            and (self.last_telemetry_at is None
                 or now - self.last_telemetry_at >= TELEMETRY_STALE_SECONDS)
        )
        lease_expired = self.command_deadline > 0.0 and now >= self.command_deadline
        owner_stale = not self._owner_ready_locked(now)
        if self.serial_fault or telemetry_stale or owner_stale:
            self.control_ready = False
            if self.serial_fault or had_active_command:
                self.operation_blocked = True
            self._clear_command_locked()
        elif lease_expired:
            self.operation_blocked = True
            self._clear_command_locked()
        return telemetry_stale

    def command_for_write(self):
        with self.lock:
            self._expire_command_locked(self.clock())
            return self.vx, self.wz

    def snapshot(self):
        with self.lock:
            telemetry_stale = self._expire_command_locked(self.clock())
            return {
                "status": self.status, "vx": int(self.vx), "wz": int(self.wz),
                "left_act": self.left_act, "right_act": self.right_act,
                "voltage_mv": self.voltage_mv,
                "ax": self.ax, "ay": self.ay, "az": self.az,
                "gx": self.gx, "gy": self.gy, "gz": self.gz,
                "rx_ok": self.rx_ok, "sim": self.sim,
                "telemetry_stale": telemetry_stale,
                "serial_fault": self.serial_fault,
                "control_ready": self.control_ready,
                "owner": self.owner, "owner_status": self.owner_status,
                "expected_source": self.expected_source,
                "owner_stale": self.last_owner_at is None or self.clock() - self.last_owner_at >= TELEMETRY_STALE_SECONDS,
            }


def validate_command(payload):
    if (not isinstance(payload, dict)
            or isinstance(payload.get("vx"), bool)
            or isinstance(payload.get("wz"), bool)):
        raise ValueError("vx and wz must be finite numbers")
    try:
        vx = float(payload["vx"])
        wz = float(payload["wz"])
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError("vx and wz must be finite numbers")
    if not math.isfinite(vx) or not math.isfinite(wz):
        raise ValueError("vx and wz must be finite numbers")
    if not -MAX_LINEAR <= vx <= MAX_LINEAR:
        raise ValueError("vx is outside +/- %d mm/s" % MAX_LINEAR)
    if not -MAX_ANGULAR <= wz <= MAX_ANGULAR:
        raise ValueError("wz is outside +/- %d mrad/s" % MAX_ANGULAR)
    return vx, wz


def reader_loop(ser, st, sim, stop_event):
    buf = bytearray()
    while not stop_event.is_set():
        if sim:
            with st.lock:
                t = time.time()
                st.status = 0x21
                st.left_act = int(300 * (1 + 0.3 * math.sin(t)))
                st.right_act = int(300 * (1 + 0.3 * math.cos(t)))
                st.voltage_mv = 12340
                st.ax, st.ay, st.az = 12, -8, 1000
                st.gx, st.gy, st.gz = 3, -4, int(50 * math.sin(t))
                st.rx_ok += 1
                st.last_telemetry_at = st.clock()
                st.owner = st.expected_source or 3
                st.owner_status = 1
                st.last_owner_at = st.clock()
                st.control_ready = True
                st.serial_fault = False
            stop_event.wait(0.05)
            continue
        try:
            chunk = ser.read(64)
        except Exception:
            st.serial_failed()
            stop_event.wait(0.1)
            continue
        if not chunk:
            continue
        buf.extend(chunk)
        consume_feedback(buf, st)


def consume_feedback(buf, st):
    """Consume mixed v1.2 telemetry/owner byte streams, retaining partial frames."""
    while buf:
        if buf[0] == TELEM_HEADER:
            size, tail = TELEM_SIZE, TELEM_TAIL
        elif buf[0] == OWNER_HEADER:
            size, tail = OWNER_SIZE, OWNER_TAIL
        else:
            del buf[0]
            continue
        if len(buf) < size:
            return
        frame = buf[:size]
        checksum = 0
        for value in frame[:-2]:
            checksum ^= value
        valid = frame[-1] == tail and checksum == frame[-2]
        if size == OWNER_SIZE:
            valid = (valid and frame[1] == 1 and frame[2] in (0, 3, 4)
                     and frame[3] <= 4)
        if not valid:
            del buf[0]
            continue
        del buf[:size]
        if size == OWNER_SIZE:
            st.apply_owner(frame)
        else:
            st.apply_telemetry(frame)



def writer_loop(ser, st, sim, stop_event, period=WRITER_PERIOD_SECONDS):
    if not sim:
        zero = build_command(0, 0)
        for _ in range(5):
            if stop_event.is_set():
                break
            try:
                ser.write(zero)
            except Exception:
                st.serial_failed()
                break
            stop_event.wait(period)
    while not stop_event.is_set():
        vx, wz = st.command_for_write()
        if not sim:
            try:
                ser.write(build_command(vx, wz))
            except Exception:
                st.serial_failed()
        stop_event.wait(period)
    if not sim:
        zero = build_command(0, 0)
        for _ in range(SHUTDOWN_ZERO_FRAMES):
            try:
                ser.write(zero)
            except Exception:
                break


PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<title>底盘控制</title><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<style>
:root{--bg:#0f1115;--card:#181b22;--line:#262b35;--fg:#e7ecf3;--dim:#8b95a7;--ok:#3ddc84;--warn:#ffc857;--err:#ff5c5c;--accent:#4ea1ff}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,"Segoe UI",sans-serif;touch-action:none}
.wrap{max-width:820px;margin:0 auto;padding:16px}
h1{font-size:18px;margin:0 0 10px;font-weight:600}
.badges{display:flex;flex-wrap:wrap;gap:6px;margin:6px 0 12px}
.badge{padding:3px 9px;border-radius:999px;border:1px solid var(--line);color:var(--dim);font-size:12px}
.badge.on{color:#0b0d10;background:var(--ok);border-color:var(--ok)}
.badge.warn.on{background:var(--warn);border-color:var(--warn)}
.badge.err.on{background:var(--err);border-color:var(--err);color:#fff}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.k{color:var(--dim);font-size:12px;margin-bottom:4px}
.v{font-size:22px;font-variant-numeric:tabular-nums;font-weight:600}
.mono{font-variant-numeric:tabular-nums}
.bar{height:6px;background:#0b0d10;border-radius:6px;overflow:hidden;margin-top:6px}
.bar i{display:block;height:100%;background:var(--accent)}
.pad{display:flex;gap:20px;align-items:center;justify-content:center;flex-wrap:wrap;margin:10px 0}
#stick{position:relative;width:220px;height:220px;border-radius:50%;background:radial-gradient(circle at 50% 50%,#20242e,#141820);border:1px solid var(--line);touch-action:none}
#stick::before{content:"";position:absolute;inset:22px;border:1px dashed #333c49;border-radius:50%}
#knob{position:absolute;left:50%;top:50%;width:84px;height:84px;margin:-42px 0 0 -42px;border-radius:50%;background:linear-gradient(180deg,#4ea1ff,#2b6fd6);box-shadow:0 6px 16px rgba(0,0,0,.4)}
.dpad{display:grid;grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,64px);gap:6px;touch-action:none}
.dpad button{width:64px;height:64px;font-size:22px;border-radius:12px;background:#222835;color:var(--fg);border:1px solid var(--line)}
.dpad button.active{background:var(--accent);color:#04121f}
.ctl{display:grid;grid-template-columns:84px 1fr 56px;align-items:center;gap:10px;margin:8px 0}
input[type=range]{width:100%}
button{background:#222835;color:var(--fg);border:1px solid var(--line);border-radius:8px;padding:8px 12px;cursor:pointer;font-size:13px}
button.dan{background:#3a1f22;border-color:#5a2b30}
.hint{color:var(--dim);font-size:12px;margin-top:8px}
</style></head><body><div class="wrap">
<h1>底盘控制 <span id="link" class="badge">连接中…</span></h1>
<div class="badges" id="badges"></div>
<div>控制权：<span id="owner_label">未知</span></div>
<div class="grid" style="margin-top:10px">
  <div class="card"><div class="k">电压</div><div class="v"><span id="volt">--</span> V</div></div>
  <div class="card"><div class="k">指令 速度 / 角速度</div><div class="v mono"><span id="cmd">0 / 0</span></div></div>
  <div class="card"><div class="k">左轮 实测 mm/s</div><div class="v mono"><span id="lval">--</span></div><div class="bar"><i id="lbar" style="width:0"></i></div></div>
  <div class="card"><div class="k">右轮 实测 mm/s</div><div class="v mono"><span id="rval">--</span></div><div class="bar"><i id="rbar" style="width:0"></i></div></div>
  <div class="card"><div class="k">加速度 X / Y / Z</div><div class="v mono" style="font-size:18px"><span id="acc">-- / -- / --</span></div></div>
  <div class="card"><div class="k">角速度 X / Y / Z</div><div class="v mono" style="font-size:18px"><span id="gyr">-- / -- / --</span></div></div>
</div>
<div class="card" style="margin-top:10px">
  <div class="k">控制：摇杆拖动 / 方向键按住 / 键盘 ↑↓←→ 或 WASD；松开即停（保留控制权）</div>
  <div class="pad">
    <div id="stick"><div id="knob"></div></div>
    <div class="dpad">
      <span></span><button data-jx="0" data-jy="-1">&#9650;</button><span></span>
      <button data-jx="-1" data-jy="0">&#9664;</button><button data-stop="1">&#9632;</button><button data-jx="1" data-jy="0">&#9654;</button>
      <span></span><button data-jx="0" data-jy="1">&#9660;</button><span></span>
    </div>
  </div>
  <div class="ctl"><span>最大前进</span><input id="maxv" type="range" min="60" max="700" step="20" value="300"><span id="maxvv" class="mono">300</span></div>
  <div class="ctl"><span>最大转向</span><input id="maxw" type="range" min="100" max="1000" step="50" value="600"><span id="maxwv" class="mono">600</span></div>
  <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:8px"><button class="dan" id="release">释放控制权</button></div>
  <div class="hint">提示：先把速度调小试；摇杆朝上=前进、朝左=左转。松开摇杆/按键立即停（保留控制权，不释放）。</div>
</div>
</div>
<script>
const B=[["ARMED","已解锁",0,0],["TIMEOUT","超时",1,1],["RX_ERR","接收错误",2,2],["WAIT","串口待解锁",3,1],["STOP","停机",4,1],["ENABLED","使能",5,0],["LOW_V","低压",6,1],["ERR","自检错误",7,2]];
function badges(s){return B.map(function(a){var on=(s>>a[2])&1;var cls=on?(a[3]==2?"badge err on":a[3]==1?"badge warn on":"badge on"):"badge";return '<span class="'+cls+'">'+a[1]+'</span>';}).join("");}
const $=function(i){return document.getElementById(i);};
let maxV=300,maxW=600,jx=0,jy=0,active=false,renew=null;
function post(vx,wz){fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({vx:vx,wz:wz})}).catch(function(){});}
function publish(){post(Math.round(-jy*maxV),Math.round(-jx*maxW));}
function start(){active=true;if(renew===null)renew=setInterval(function(){if(active)publish();},80);publish();}
function stopHeld(){active=false;if(renew!==null){clearInterval(renew);renew=null;}post(0,0);}
function releaseAll(){stopHeld();fetch('/stop',{method:'POST',keepalive:true});}
function setVec(nx,ny){jx=nx;jy=ny;if(active)publish();}
const stick=$('stick'),knob=$('knob');let stickOn=false;
function jmove(e){var r=stick.getBoundingClientRect();var dx=(e.clientX-(r.left+r.width/2))/(r.width/2);var dy=(e.clientY-(r.top+r.height/2))/(r.height/2);var m=Math.hypot(dx,dy);if(m>1){dx/=m;dy/=m;}knob.style.transform='translate('+(dx*66)+'px,'+(dy*66)+'px)';setVec(dx,dy);}
stick.addEventListener('pointerdown',function(e){e.preventDefault();stickOn=true;try{stick.setPointerCapture(e.pointerId);}catch(x){}jmove(e);start();});
stick.addEventListener('pointermove',function(e){if(stickOn)jmove(e);});
['pointerup','pointercancel','lostpointercapture'].forEach(function(n){stick.addEventListener(n,function(){stickOn=false;knob.style.transform='translate(0,0)';setVec(0,0);stopHeld();});});
document.querySelectorAll('.dpad button').forEach(function(b){
  function setOn(v){b.classList.toggle('active',v);}
  b.addEventListener('pointerdown',function(e){e.preventDefault();try{b.setPointerCapture(e.pointerId);}catch(x){}setOn(true);if(b.dataset.stop){setVec(0,0);stopHeld();return;}setVec(+b.dataset.jx,+b.dataset.jy);start();});
  ['pointerup','pointercancel','lostpointercapture'].forEach(function(n){b.addEventListener(n,function(){setOn(false);if(!b.dataset.stop)stopHeld();});});
});
var keys={},KM={ArrowUp:['jy',-1],KeyW:['jy',-1],ArrowDown:['jy',1],KeyS:['jy',1],ArrowLeft:['jx',-1],KeyA:['jx',-1],ArrowRight:['jx',1],KeyD:['jx',1]};
function fromKeys(){var nx=0,ny=0;for(var k in keys){var a=KM[k];if(a[0]==='jx')nx+=a[1];else ny+=a[1];}nx=Math.max(-1,Math.min(1,nx));ny=Math.max(-1,Math.min(1,ny));if(nx||ny){setVec(nx,ny);start();}else stopHeld();}
document.addEventListener('keydown',function(e){if(KM[e.code]){e.preventDefault();keys[e.code]=1;fromKeys();}else if(e.code==='Space'){e.preventDefault();setVec(0,0);stopHeld();}});
document.addEventListener('keyup',function(e){if(KM[e.code]){delete keys[e.code];fromKeys();}});
$('maxv').oninput=function(e){maxV=+e.target.value;$('maxvv').textContent=maxV;if(active)publish();};
$('maxw').oninput=function(e){maxW=+e.target.value;$('maxwv').textContent=maxW;if(active)publish();};
$('release').onclick=releaseAll;
['blur','pagehide'].forEach(function(n){window.addEventListener(n,function(){setVec(0,0);stopHeld();});});
document.addEventListener('visibilitychange',function(){if(document.hidden){setVec(0,0);stopHeld();}});
window.addEventListener('beforeunload',function(){navigator.sendBeacon('/stop','');});
async function tick(){try{var d=await (await fetch('/state')).json();
  link.className=d.telemetry_stale?'badge warn on':'badge on';
  link.textContent=d.sim?'模拟数据':(d.telemetry_stale?'回执超时':'已连接');
  badges_el.innerHTML=badges(d.status);
  volt.textContent=(d.voltage_mv/1000).toFixed(2);
  cmd.textContent=d.vx+' / '+d.wz;
  lval.textContent=d.left_act;rval.textContent=d.right_act;
  lbar.style.width=Math.min(100,Math.abs(d.left_act)/7)+'%';
  rbar.style.width=Math.min(100,Math.abs(d.right_act)/7)+'%';
  acc.textContent=d.ax+' / '+d.ay+' / '+d.az;
  gyr.textContent=d.gx+' / '+d.gy+' / '+d.gz;
  owner_label.textContent=d.owner_stale?"控制权回执超时":({0:"无人控制",3:"UART3",4:"蓝牙"}[d.owner]||"未知");
}catch(e){link.className='badge err on';link.textContent='未连接';}}
const badges_el=$('badges');
setInterval(tick,200);tick();
</script></body></html>"""


def make_handler(st):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _json(self, obj, status=200):
            b = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path == "/" or self.path.startswith("/index"):
                b = PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
            elif self.path.startswith("/state"):
                self._json(st.snapshot())
            else:
                self.send_error(404)

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(n) if n else b"{}"
            try:
                d = json.loads(body or b"{}")
            except Exception:
                d = {}
            if self.path.startswith("/stop"):
                st.clear_command(release=True)
                self._json({"ok": True})
            elif self.path.startswith("/cmd"):
                try:
                    vx, wz = validate_command(d)
                except ValueError as exc:
                    st.clear_command()
                    self._json({"ok": False, "error": str(exc)}, status=400)
                    return
                if not st.set_command(vx, wz):
                    self._json({"ok": False, "error": "control is not ready; release and press run again"}, status=409)
                    return
                self._json({"ok": True})
            else:
                self.send_error(404)

    return H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--source", choices=("uart3", "bluetooth"), default="uart3",
                    help="v1.2 control source; requires owner feedback")
    ap.add_argument("--http", type=int, default=8000)
    ap.add_argument("--sim", action="store_true", help="no serial, fake telemetry")
    args = ap.parse_args()

    st = State(sim=args.sim, expected_source=3 if args.source == "uart3" else 4)
    if args.baud is None:
        args.baud = 115200 if args.source == "uart3" else 9600
    ser = None
    if not args.sim:
        import serial
        ser = serial.Serial()
        ser.port = args.port
        ser.baudrate = args.baud
        ser.bytesize = 8
        ser.parity = "N"
        ser.stopbits = 1
        ser.timeout = 0.05
        ser.write_timeout = 0.1
        ser.dtr = False
        ser.rts = False
        ser.open()
        print("opened %s @ %d" % (args.port, args.baud))

    stop_event = threading.Event()
    reader = threading.Thread(target=reader_loop, args=(ser, st, args.sim, stop_event), daemon=True)
    writer = threading.Thread(target=writer_loop, args=(ser, st, args.sim, stop_event), daemon=True)
    reader.start()
    writer.start()

    srv = ThreadingHTTPServer(("127.0.0.1", args.http), make_handler(st))
    print("dashboard:  http://127.0.0.1:%d/" % args.http)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        st.clear_command(release=True)
        stop_event.set()
        writer.join(timeout=0.5)
        reader.join(timeout=0.2)
        if ser:
            ser.close()


if __name__ == "__main__":
    main()
