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
    def __init__(self, clock=time.monotonic, sim=False):
        self.lock = threading.Lock()
        self.clock = clock
        self.sim = sim
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
            self.control_ready = (
                bool(self.status & STATUS_ARMED)
                and bool(self.status & STATUS_ENABLED)
                and not bool(self.status & STATUS_CONTROL_FAULTS)
            )
            if not self.control_ready:
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
        if self.serial_fault or telemetry_stale:
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
        while True:
            i = buf.find(bytes([TELEM_HEADER]))
            if i < 0:
                buf.clear()
                break
            if len(buf) - i < TELEM_SIZE:
                del buf[:i]
                break
            frame = buf[i:i + TELEM_SIZE]
            del buf[:i + TELEM_SIZE]
            if frame[TELEM_SIZE - 1] != TELEM_TAIL:
                continue
            cks = 0
            for b in frame[:TELEM_SIZE - 2]:
                cks ^= b
            if cks != frame[TELEM_SIZE - 2]:
                continue
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
<title>底盘状态</title><meta name="viewport" content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0f1115;--card:#181b22;--line:#262b35;--fg:#e7ecf3;--dim:#8b95a7;--ok:#3ddc84;--warn:#ffc857;--err:#ff5c5c;--accent:#4ea1ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.5 system-ui,"Segoe UI",sans-serif}
.wrap{max-width:900px;margin:0 auto;padding:20px}
h1{font-size:18px;margin:0 0 14px;font-weight:600}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}
.k{color:var(--dim);font-size:12px;margin-bottom:6px}
.v{font-size:26px;font-variant-numeric:tabular-nums;font-weight:600}
.badges{display:flex;flex-wrap:wrap;gap:8px;margin:6px 0 16px}
.badge{padding:4px 10px;border-radius:999px;border:1px solid var(--line);color:var(--dim);font-size:12px}
.badge.on{color:#0b0d10;background:var(--ok);border-color:var(--ok)}
.badge.warn.on{background:var(--warn);border-color:var(--warn)}
.badge.err.on{background:var(--err);border-color:var(--err);color:#fff}
.ctl{display:grid;grid-template-columns:70px 1fr 70px;align-items:center;gap:10px;margin:10px 0}
input[type=range]{width:100%}
button{background:#222835;color:var(--fg);border:1px solid var(--line);border-radius:8px;padding:8px 12px;cursor:pointer;font-size:13px}
button:hover{border-color:var(--accent)}
button.pri{background:var(--accent);border-color:var(--accent);color:#04121f;font-weight:600}
button.dan{background:#3a1f22;border-color:#5a2b30}
.bar{height:8px;background:#0b0d10;border-radius:6px;overflow:hidden;margin-top:8px}
.bar i{display:block;height:100%;background:var(--accent)}
.mono{font-variant-numeric:tabular-nums}
</style></head><body><div class="wrap">
<h1>底盘状态 <span id="link" class="badge">连接中…</span></h1>
<div class="badges" id="badges"></div>
<div class="grid">
  <div class="card"><div class="k">电压</div><div class="v"><span id="volt">--</span> V</div></div>
  <div class="card"><div class="k">指令 速度 / 角速度</div><div class="v mono"><span id="cmd">0 / 0</span></div></div>
  <div class="card"><div class="k">左轮 实测 mm/s</div>
     <div class="v mono"><span id="lval">--</span></div><div class="bar"><i id="lbar" style="width:0"></i></div></div>
  <div class="card"><div class="k">右轮 实测 mm/s</div>
     <div class="v mono"><span id="rval">--</span></div><div class="bar"><i id="rbar" style="width:0"></i></div></div>
  <div class="card"><div class="k">加速度 X / Y / Z</div><div class="v mono" style="font-size:20px"><span id="acc">-- / -- / --</span></div></div>
  <div class="card"><div class="k">角速度 X / Y / Z</div><div class="v mono" style="font-size:20px"><span id="gyr">-- / -- / --</span></div></div>
  <div class="card"><div class="k">接收帧数</div><div class="v mono" id="rx">0</div></div>
</div>
<div class="card" style="margin-top:12px">
  <div class="k">控制</div>
  <div class="ctl"><span>前进 mm/s</span><input id="vx" type="range" min="-700" max="700" step="10" value="0"><span id="vxv" class="mono">0</span></div>
  <div class="ctl"><span>转向 mrad/s</span><input id="wz" type="range" min="-1000" max="1000" step="20" value="0"><span id="wzv" class="mono">0</span></div>
  <div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:8px">
    <button onclick="send()">发送一次（0.5秒）</button>
    <button id="run" class="pri">按住运行</button>
    <button class="dan" onclick="stop()">停止</button>
    <button onclick="quick(200,0)">前进</button>
    <button onclick="quick(-200,0)">后退</button>
    <button onclick="quick(0,500)">左转</button>
    <button onclick="quick(0,-500)">右转</button>
    <button onclick="quick(0,0)">归零</button>
  </div>
</div>
</div>
<script>
const B=[["ARMED","已解锁",0,0],["TIMEOUT","超时",1,1],["RX_ERR","接收错误",2,2],["WAIT","串口待解锁",3,1],["STOP","停机",4,1],["ENABLED","使能",5,0],["LOW_V","低压",6,1],["ERR","自检错误",7,2]];
function badges(s){
  return B.map(([n,zh,bit,sev])=>{
    const on=(s>>bit)&1;const cls=on?(sev==2?"badge err on":sev==1?"badge warn on":"badge on"):"badge";
    return `<span class="${cls}">${zh}</span>`;}).join("");
}
async function tick(){
  try{
    const r=await fetch('/state');const d=await r.json();
    link.className=d.telemetry_stale?'badge warn on':'badge on';
    link.textContent=d.sim?'模拟数据':(d.telemetry_stale?'回执超时':'已连接');
    badges_el.innerHTML=badges(d.status);
    volt.textContent=(d.voltage_mv/1000).toFixed(2);
    cmd.textContent=d.vx+' / '+d.wz;
    lval.textContent=d.left_act;
    rval.textContent=d.right_act;
    lbar.style.width=Math.min(100,Math.abs(d.left_act)/7)+'%';
    rbar.style.width=Math.min(100,Math.abs(d.right_act)/7)+'%';
    acc.textContent=d.ax+' / '+d.ay+' / '+d.az;
    gyr.textContent=d.gx+' / '+d.gy+' / '+d.gz;
    rx.textContent=d.rx_ok;
  }catch(e){link.className='badge err on';link.textContent='未连接';}
}
const badges_el=document.getElementById('badges');
['vx','wz'].forEach(id=>{const el=document.getElementById(id),out=document.getElementById(id+'v');
  el.oninput=()=>out.textContent=el.value;});
async function post(vx,wz){await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({vx:+vx,wz:+wz})});}
function send(){post(+vx.value,+wz.value);}
function quick(a,b){vx.value=a;wz.value=b;document.getElementById('vxv').textContent=a;document.getElementById('wzv').textContent=b;post(a,b);}
function stop(){vx.value=0;wz.value=0;document.getElementById('vxv').textContent=0;document.getElementById('wzv').textContent=0;fetch('/stop',{method:'POST',keepalive:true});}
let renew=null;
function startRun(e){e.preventDefault();if(renew!==null)return;if(e.pointerId!==undefined)run.setPointerCapture(e.pointerId);send();renew=setInterval(send,100);}
function endRun(){if(renew!==null){clearInterval(renew);renew=null;}stop();}
const run=document.getElementById('run');
run.addEventListener('pointerdown',startRun);
['pointerup','pointercancel'].forEach(n=>window.addEventListener(n,endRun));
run.addEventListener('lostpointercapture',endRun);
document.addEventListener('visibilitychange',()=>{if(document.hidden)endRun();});
window.addEventListener('blur',endRun);
window.addEventListener('pagehide',endRun);
window.addEventListener('beforeunload',()=>navigator.sendBeacon('/stop',''));
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
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http", type=int, default=8000)
    ap.add_argument("--sim", action="store_true", help="no serial, fake telemetry")
    args = ap.parse_args()

    st = State(sim=args.sim)
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
