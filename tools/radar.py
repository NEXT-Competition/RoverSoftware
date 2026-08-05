"""Live radar-style view of the ultrasonic, in a browser.

    python tools/radar.py --pins 27,22
    # then open http://<robot>:8000 from anything on the same network

The companion to tools/ultrasonic_monitor.py rather than a replacement for it:
the monitor is the numbers and the guard thresholds, which is what you want when
sizing a stop distance. This is the shape of the room, which is what you want
when deciding where to MOUNT the module — a sensor aimed slightly down finds the
floor at a fixed range, and that reads as a wall on a number and as an obvious
arc here.

Deliberately NOT wired to the robot's config or its Ultrasonic class: this runs
on a Pi with nothing else installed and no service configured, which is exactly
the state a module is in when somebody is still deciding where to bolt it.

ONE FIXED SENSOR MEASURES RANGE, NOT BEARING. The blip always sits on the centre
line and the shaded wedge is the ~15 degrees of arc the echo could have come
from. Drawing it as a point at a bearing nobody measured would be a lie that
looks precise. Mount the module on a servo and sweep it if you want position.

Only one process may hold the trigger pin. The robot service claims these pins
at start-up whenever robot.env names them, so stop it first:
    sudo systemctl stop roversoftware-robot
Two readers both time out and both report no echo, which looks exactly like a
sensor that is not wired.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:  # optional on a dev laptop, as it is for the motors and the encoders
    from fusion_hat.modules import Ultrasonic
    from fusion_hat.pin import Pin
except Exception as _e:  # pragma: no cover - hardware-only import
    Ultrasonic = Pin = None
    _IMPORT_ERROR = _e
else:
    _IMPORT_ERROR = None


def _pins(text):
    try:
        trig, echo = (int(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(
            "expected two BCM pin numbers, e.g. --pins 27,22") from None
    return trig, echo


p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
p.add_argument("--pins", type=_pins, default=(27, 22), metavar="TRIG,ECHO",
               help="BCM GPIO pins for TRIG and ECHO (default 27,22)")
p.add_argument("--port", type=int, default=8000, help="HTTP port (default 8000)")
p.add_argument("--max-cm", type=float, default=200.0,
               help="outer ring of the display, in cm (default 200)")
p.add_argument("--scale", type=float, default=1.0,
               help="correction factor if readings run long or short; the "
                    "library assumes 343.3 m/s, i.e. about 20 C, and the speed "
                    "of sound moves ~0.6%% per 10 C")
args = p.parse_args()

SCALE = args.scale
MAX_CM = args.max_cm

TRIG, ECHO = args.pins
if Ultrasonic is None:
    raise SystemExit(
        f"[radar] fusion_hat is not installed ({_IMPORT_ERROR}). This tool "
        "reads the sensor through the Fusion HAT library and only runs on the "
        "robot; install it with `just bootstrap` (or SunFounder's install.sh).")

sensor = Ultrasonic(Pin(TRIG), Pin(ECHO))
sensor.start_thread(0.05)   # background reads, so HTTP never blocks on a ping

PAGE = """<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>ultrasonic radar</title>
<style>body{margin:0;background:#04140a;display:grid;place-items:center;height:100vh;
font:14px ui-monospace,monospace;color:#4f9}canvas{max-width:100vw;max-height:80vh}
#r{padding:.6em;letter-spacing:.1em}</style>
<canvas id=c width=520 height=300></canvas><div id=r>--</div><script>
const c=document.getElementById('c'),x=c.getContext('2d'),MAX=%d;
const CX=260,CY=280,R=250,HALF=7.5*Math.PI/180;
let hist=[];
function draw(d){
  x.fillStyle='#04140a';x.fillRect(0,0,520,300);
  x.strokeStyle='#0f5';x.globalAlpha=.25;
  for(let i=1;i<=4;i++){x.beginPath();x.arc(CX,CY,R*i/4,Math.PI,0);x.stroke();
    x.fillStyle='#0f5';x.fillText((MAX*i/4)+'cm',CX+4,CY-R*i/4+12);}
  x.beginPath();x.moveTo(CX-R,CY);x.lineTo(CX+R,CY);x.stroke();
  x.globalAlpha=1;
  // beam cone: where the object could be, given we only know range
  x.fillStyle='rgba(0,255,120,.07)';x.beginPath();x.moveTo(CX,CY);
  x.arc(CX,CY,R,-Math.PI/2-HALF,-Math.PI/2+HALF);x.fill();
  hist=hist.filter(h=>Date.now()-h.t<1500);
  for(const h of hist){
    const y=CY-Math.min(h.d/MAX,1)*R;
    x.globalAlpha=1-(Date.now()-h.t)/1500;
    x.fillStyle='#5f9';x.beginPath();x.arc(CX,y,4,0,7);x.fill();
  }
  x.globalAlpha=1;
  if(d>0){const y=CY-Math.min(d/MAX,1)*R;
    x.fillStyle='#9ff';x.beginPath();x.arc(CX,y,7,0,7);x.fill();
    x.fillStyle='#04140a';x.fillText(d.toFixed(0),CX-8,y+4);}
}
async function tick(){
  let d=-1;
  try{d=parseFloat(await(await fetch('/d')).text());}catch(e){}
  document.getElementById('r').textContent=d>0?d.toFixed(1)+' cm':'no echo';
  if(d>0)hist.push({d:d,t:Date.now()});
  draw(d);setTimeout(tick,100);
}
tick();</script>""" % MAX_CM


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/d':
            body, ctype = str(sensor.read() * SCALE), 'text/plain'
        else:
            body, ctype = PAGE, 'text/html'
        body = body.encode()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


if __name__ == '__main__':
    print(f'TRIG=GPIO{TRIG} ECHO=GPIO{ECHO} -> http://0.0.0.0:{args.port}')
    print('A blip that never appears is a sensor that is not wired, NOT a room '
          'with nothing in it.')
    ThreadingHTTPServer(('0.0.0.0', args.port), H).serve_forever()
