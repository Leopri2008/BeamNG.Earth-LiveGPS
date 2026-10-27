"""BeamNG.Earth Live GPS - PC relay.

Reads the live position of your car in BeamNG.Earth without changing the mod or the server EXE.

    Game (Lua mod) --TCP--> this relay :47614 --TCP--> BeamNG.Earth server (EXE) :47613

The mod sends a 'pose' message to its server about twice per second, which already contains the car's
real latitude/longitude and speed. This relay passes everything through unchanged and listens in.

Usage:
  1. Start the BeamNG.Earth server (EXE) as usual (listens on 127.0.0.1:47613).
  2. Run this script:  python bne_live_coords.py
  3. Start BeamNG.drive with the launch option  -bngeport 47614
     (Steam > BeamNG.drive > Properties > Launch options).
  4. Live data is available in:
       - this console
       - live_position.json  (next to this script, rewritten on every pose)
       - http://<PC-IP>:47700/   (JSON, reachable on your Wi-Fi, used by the BNE Mock GPS app)

Part of BeamNG.Earth Live GPS. See LICENSE for terms.
"""
import json
import math
import os
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

VERSION = "1.0"

LISTEN_PORT = 47614       # the game connects here (-bngeport 47614)
SERVER_PORT = 47613       # the BeamNG.Earth server listens here (default)
HTTP_PORT = 47700         # JSON endpoint for the phone app / your own tools
OUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "live_position.json")
MAX_FRAME = 16 * 1024 * 1024

DEBUG = True              # show which message types the game sends, until the first pose arrives

latest = {}
seen = {}
lock = threading.Lock()


def handle_pose(msg):
    global DEBUG
    DEBUG = False
    vx, vy = msg.get("vx") or 0.0, msg.get("vy") or 0.0     # m/s in the local frame: x = east, y = north
    pos = {
        "lat": msg.get("lat"),
        "lon": msg.get("lon"),
        "speed_kmh": round((msg.get("speed") or 0) * 3.6, 1),
        "speed_ms": msg.get("speed") or 0.0,
        "ve": vx,
        "vn": vy,
        "bearing": (math.degrees(math.atan2(vx, vy)) + 360.0) % 360.0,
        "z_local": msg.get("z"),
        "anchorId": msg.get("anchorId"),
        "t": time.time(),
    }
    if pos["lat"] is None:
        return
    with lock:
        latest.clear()
        latest.update(pos)
    try:
        tmp = OUT_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(pos, f)
        os.replace(tmp, OUT_FILE)
    except OSError:
        pass
    print(f"\r  LIVE  {pos['lat']:.6f}, {pos['lon']:.6f}   {pos['speed_kmh']:6.1f} km/h   "
          f"{pos['bearing']:5.1f} deg   ", end="", flush=True)


def pipe(src, dst, inspect):
    buf = bytearray()
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)                 # forward unchanged first
            if not inspect:
                continue
            buf += data                       # then listen in: 4-byte LE length + JSON
            while len(buf) >= 4:
                n = struct.unpack_from("<I", buf)[0]
                if n > MAX_FRAME:             # lost sync -> drop the buffer
                    buf.clear()
                    break
                if len(buf) < 4 + n:
                    break
                payload = bytes(buf[4:4 + n])
                del buf[:4 + n]
                try:
                    msg = json.loads(payload)   # field order from Lua is random -> always parse fully
                except ValueError:
                    continue
                t = msg.get("type") if isinstance(msg, dict) else None
                seen[t] = seen.get(t, 0) + 1
                if t == "pose":
                    handle_pose(msg)
                elif DEBUG:
                    print(f"\n  [debug] messages from the game so far: {seen}", flush=True)
    except OSError:
        pass
    finally:
        for s in (src, dst):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def serve_relay():
    ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        ls.bind(("127.0.0.1", LISTEN_PORT))
    except OSError as e:
        print(f"  ERROR: port {LISTEN_PORT} is already in use ({e}). Is the script already running?")
        return
    ls.listen(4)
    print("  Waiting for BeamNG.drive (launch option -bngeport 47614)...")
    while True:
        game, _ = ls.accept()
        try:
            srv = socket.create_connection(("127.0.0.1", SERVER_PORT), timeout=5)
            srv.settimeout(None)
        except OSError as e:
            print(f"\n  ERROR: BeamNG.Earth server not reachable ({e}). Is the BeamNG.Earth EXE running?")
            game.close()
            continue
        print("\n  Game connected. Load into the world and drive.")
        threading.Thread(target=pipe, args=(game, srv, True), daemon=True).start()
        threading.Thread(target=pipe, args=(srv, game, False), daemon=True).start()


class Http(BaseHTTPRequestHandler):
    def do_GET(self):
        with lock:
            body = json.dumps(latest).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


def lan_ip():
    """The PC's IP address on the local network (for the phone)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.168.0.1", 9))     # sends nothing, only picks the right network adapter
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    try:
        http = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Http)
    except OSError as e:
        print(f"ERROR: port {HTTP_PORT} is already in use ({e}). Is the script already running?")
        return
    threading.Thread(target=http.serve_forever, daemon=True).start()

    line = "=" * 62
    print(line)
    print(f"  BeamNG.Earth Live GPS  v{VERSION}")
    print(line)
    print(f"  Phone app address:  {lan_ip()}:{HTTP_PORT}")
    print(f"  Local JSON:         http://127.0.0.1:{HTTP_PORT}/")
    print(f"  File:               {OUT_FILE}")
    print(f"  Relay:              game -> :{LISTEN_PORT} -> server :{SERVER_PORT}")
    print(line)
    serve_relay()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Stopped.")
