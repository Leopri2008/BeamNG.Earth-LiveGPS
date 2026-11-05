"""BeamNG.Earth Live GPS - PC relay.

Reads the live position of your car in BeamNG.Earth without changing the mod or the server EXE.

    Game (Lua mod) --TCP--> this relay :47614 --TCP--> BeamNG.Earth server (EXE) :47613

The mod sends a 'pose' message to its server about twice per second, which already contains the car's
real latitude/longitude and speed. This relay passes everything through unchanged and listens in.

The script also records your drive (works with the browser closed):
  - the driven route, kept across restarts until you reset it (export as GPX or KML)
  - trip stats (distance, drive time, average and top speed)
  - a lifetime odometer: only real driving counts, never teleports or glitches
  - the fog of the world: every place you really drove through is revealed on the map, kept until you reset it

On start it looks once for a newer release on GitHub (start with --no-update-check to skip that).

Usage:
  1. Start the BeamNG.Earth server (EXE) as usual (listens on 127.0.0.1:47613).
  2. Run this script:  python bne_live_coords.py
  3. Start BeamNG.drive with the launch option  -bngeport 47614
     (Steam > BeamNG.drive > Properties > Launch options).
  4. Live data is available in:
       - this console
       - live_position.json  (next to this script, rewritten on every pose)
       - http://<PC-IP>:47700/            (JSON, reachable on your Wi-Fi, used by the BNE Mock GPS app)
       - http://<PC-IP>:47700/map         (live map in any browser: iPhone, iPad, Android, PC, OBS)
       - http://<PC-IP>:47700/export.gpx  (driven route as GPX; /export.kml for Google Earth)

Part of BeamNG.Earth Live GPS. See LICENSE for terms.
"""
import atexit
import json
import math
import os
import re
import socket
import struct
import sys
import threading
import time
import urllib.request
import uuid
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from xml.sax.saxutils import escape

VERSION = "1.3"

LISTEN_PORT = 47614       # the game connects here (-bngeport 47614)
SERVER_PORT = 47613       # the BeamNG.Earth server listens here (default)
HTTP_PORT = 47700         # JSON endpoint for the phone app / your own tools
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE = os.path.join(SCRIPT_DIR, "live_position.json")
DATA_DIR = os.path.join(SCRIPT_DIR, "BNE_LiveGPS_data")   # route + stats, kept across restarts
TRACK_FILE = os.path.join(DATA_DIR, "track.ndjson")
STATS_FILE = os.path.join(DATA_DIR, "stats.json")
FOG_FILE = os.path.join(DATA_DIR, "fog.json")
MAX_FRAME = 64 * 1024 * 1024   # same limit as the mod (framing.lua)

DEBUG = True              # show which message types the game sends, until the first pose arrives

latest = {}
seen = {}
lock = threading.Lock()


# ===================================================================== drive recorder
#
# Every pose is checked against the last accepted one before it moves the live position, the route or any counter:
#   - DRIVING: the distance fits the car's own speed  -> route + trip + odometer
#   - JUMP:    it doesn't fit (teleport, glitch, the world reloading). The new place is only taken once several poses
#              in a row agree on it (CONFIRM_N); a single pose somewhere else (e.g. a flash to the map's zero point)
#              is thrown away. A confirmed jump starts a new route segment (no line through the world) and counts
#              nothing.
# Speeds only count for the top speed when the car's position really moved that fast.

R_EARTH = 6371008.8
CONFIRM_N = 3              # poses (~1.5 s) that must agree before a jump is believed
MAX_ACCEL = 40.0           # m/s^2 (~4 g): faster speed changes are glitches
JUMP_CHAIN_SPEED = 400.0   # m/s: poses of one confirmed jump must be this close to each other
RESUME_GAP_S = 10.0        # longer without poses (game closed, paused, script restarted) = resume, count nothing
RESUME_JOIN_M = 30.0       # after a gap, continue the same route segment if the car is still this close
POINT_MIN_M = 5.0          # route resolution
SAVE_EVERY_S = 5.0
FOG_Z = 20                 # fog grid = Web Mercator tiles at zoom 20 (~25 m squares in Europe)
FOG_N = 1 << FOG_Z
FOG_MAX_EDIT = 50000       # cells per edit request
FOG_REACH_M = 6.0          # a road along a cell edge fills both cells: cells within this distance of the car count as driven through
REPO = "Leopri2008/BeamNG.Earth-LiveGPS"
update_info = {}           # filled by check_update() when a newer release exists


def dist_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH * math.asin(min(1.0, math.sqrt(a)))


def iso(t):
    return datetime.fromtimestamp(t, timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.track_id = uuid.uuid4().hex[:12]
        self.points = []           # [lat, lon, t, speed_ms, seg]
        self.seg = 0
        self.trip = {"dist_m": 0.0, "moving_s": 0.0, "max_ms": 0.0, "started": time.time()}
        self.odo_m = 0.0
        self.last = None           # last accepted pose: dict(lat, lon, t, v)
        self.pending = []          # poses of an unconfirmed jump
        self.open_seg = False      # True once the current segment has a stored point
        self.dirty = False
        self.saved_at = 0.0
        self.glitches = 0
        self.prev_vok = 0.0
        self._load()

    # ------------------------------------------------------------------ storage
    def _load(self):
        os.makedirs(DATA_DIR, exist_ok=True)
        try:
            with open(STATS_FILE, encoding="utf-8") as f:
                s = json.load(f)
            self.odo_m = float(s.get("odo_m", 0.0))
            self.trip.update({k: float(s["trip"][k]) for k in ("dist_m", "moving_s", "max_ms", "started")
                              if k in s.get("trip", {})})
            self.track_id = s.get("track_id") or self.track_id
        except (OSError, ValueError, KeyError, TypeError):
            pass
        try:
            with open(TRACK_FILE, encoding="utf-8") as f:
                for line in f:
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue           # a line cut off by a crash
                    if isinstance(row, dict):
                        if row.get("id"):
                            self.track_id = row["id"]
                    elif isinstance(row, list) and len(row) >= 5:
                        self.points.append(row[:5])
        except OSError:
            self._write_header()
        if self.points:
            self.seg = int(self.points[-1][4])
            la, lo, t, v, _ = self.points[-1]
            self.last = {"lat": la, "lon": lo, "t": t, "v": 0.0}   # continue from here (after a gap)
            self.open_seg = True

    def _write_header(self):
        try:
            with open(TRACK_FILE, "w", encoding="utf-8") as f:
                f.write(json.dumps({"id": self.track_id, "created": time.time()}) + "\n")
        except OSError:
            pass

    def _append_point(self, row):
        self.points.append(row)
        try:
            with open(TRACK_FILE, "a", encoding="utf-8") as f:
                f.write(json.dumps(row, separators=(",", ":")) + "\n")
        except OSError:
            pass

    def save(self, force=False):
        now = time.time()
        if not (self.dirty or force) or (not force and now - self.saved_at < SAVE_EVERY_S):
            return
        data = {"version": VERSION, "track_id": self.track_id, "odo_m": round(self.odo_m, 2),
                "trip": {k: round(v, 3) for k, v in self.trip.items()}}
        try:
            tmp = STATS_FILE + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f)
            os.replace(tmp, STATS_FILE)
            self.dirty, self.saved_at = False, now
        except OSError:
            pass

    def reset_trip(self):
        with self.lock:
            self.track_id = uuid.uuid4().hex[:12]
            self.points = []
            self.seg += 1
            self.open_seg = False
            self.trip = {"dist_m": 0.0, "moving_s": 0.0, "max_ms": 0.0, "started": time.time()}
            self._write_header()
            self.dirty = True
            self.save(force=True)

    # ------------------------------------------------------------------ route points
    def _store(self, lat, lon, t, v, force=False):
        if self.points and self.open_seg and not force:
            p = self.points[-1]
            if p[4] == self.seg and dist_m(p[0], p[1], lat, lon) < POINT_MIN_M:
                return
        self._append_point([round(lat, 7), round(lon, 7), round(t, 1), round(v, 2), self.seg])
        self.open_seg = True

    def _new_segment(self):
        self.seg += 1
        self.open_seg = False

    # ------------------------------------------------------------------ the filter
    def feed(self, lat, lon, speed, now):
        """Returns 'drive', 'jump', 'resume', 'pending' or 'drop'. Only 'drive', 'jump' and 'resume' move the car."""
        if lat is None or lon is None or not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            return "drop"
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:          # null island: never a real place in BeamNG.Earth
            self.glitches += 1
            return "drop"
        speed = max(0.0, float(speed or 0.0))
        with self.lock:
            a = self.last
            if a is None:
                return self._jump_candidate(lat, lon, speed, now, first=True)
            dt = max(0.05, now - a["t"])
            d = dist_m(a["lat"], a["lon"], lat, lon)

            if dt > RESUME_GAP_S:                       # back after a pause / restart / next day
                if d <= RESUME_JOIN_M:
                    self.pending = []
                    self._accept(lat, lon, now, min(speed, 1.0))
                    return "resume"
                return self._jump_candidate(lat, lon, speed, now)

            v_cap = a["v"] + MAX_ACCEL * dt + 3.0           # the most the speed can have grown
            v_new = speed if speed <= v_cap else a["v"]     # a spike: keep the last good speed
            allowed = max(a["v"], v_new) * dt * 1.5 + 4.0
            if d <= allowed:
                self.pending = []
                v_pos = d / dt
                moving = speed > 0.3 and v_pos > 0.3
                if moving:
                    self.trip["dist_m"] += d
                    self.odo_m += d
                    self.trip["moving_s"] += min(dt, 5.0)
                    # top speed: only what the position confirms, held for two poses in a row (~1 s):
                    # a spike in the speed value alone, or in a single pose, never counts
                    v_ok = min(v_new, v_pos * 1.1 + 1.0)
                    held = min(v_ok, self.prev_vok)
                    self.prev_vok = v_ok
                    if held > self.trip["max_ms"]:
                        self.trip["max_ms"] = held
                    self.dirty = True
                if not moving:
                    self.prev_vok = 0.0
                self._accept(lat, lon, now, v_new)
                if moving or not self.open_seg:
                    self._store(lat, lon, now, v_new)
                return "drive"
            return self._jump_candidate(lat, lon, speed, now)

    def _accept(self, lat, lon, now, v):
        if self.last is None or now - self.last["t"] > RESUME_GAP_S:
            self.prev_vok = 0.0
        self.last = {"lat": lat, "lon": lon, "t": now, "v": v}

    def _jump_candidate(self, lat, lon, speed, now, first=False):
        p = {"lat": lat, "lon": lon, "t": now, "v": speed}
        if self.pending:
            q = self.pending[-1]
            dt = max(0.05, now - q["t"])
            if dist_m(q["lat"], q["lon"], lat, lon) > JUMP_CHAIN_SPEED * dt + 20.0:
                self.pending = []                  # the jump poses don't agree with each other: start over
        self.pending.append(p)
        if len(self.pending) < CONFIRM_N:
            return "pending"
        # confirmed: a real teleport (or a new start). New route segment, nothing counted.
        self.pending = []
        self._new_segment()
        self.prev_vok = 0.0
        self._accept(lat, lon, now, min(speed, 5.0))
        self._store(lat, lon, now, 0.0, force=True)
        return "jump"

    # ------------------------------------------------------------------ views
    def summary(self):
        t = self.trip
        return {
            "track_id": self.track_id, "track_n": len(self.points), "seg": self.seg,
            "trip": {"dist_m": round(t["dist_m"], 1), "moving_s": round(t["moving_s"], 1),
                     "max_ms": round(t["max_ms"], 2), "started": t["started"]},
            "odo_m": round(self.odo_m, 1),
        }

    def track_since(self, since, track_id):
        with self.lock:
            if track_id != self.track_id or since < 0 or since > len(self.points):
                since = 0
            pts = [[p[0], p[1], p[4]] for p in self.points[since:]]
            return {"id": self.track_id, "from": since, "n": len(self.points), "seg": self.seg, "pts": pts}

    def segments(self):
        with self.lock:
            segs, cur, cur_id = [], [], None
            for p in self.points:
                if p[4] != cur_id:
                    if len(cur) > 1:
                        segs.append(cur)
                    cur, cur_id = [], p[4]
                cur.append(p)
            if len(cur) > 1:
                segs.append(cur)
            return segs

    def gpx(self):
        out = ['<?xml version="1.0" encoding="UTF-8"?>',
               '<gpx version="1.1" creator="BeamNG.Earth Live GPS by Leopri2008" '
               'xmlns="http://www.topografix.com/GPX/1/1">',
               f'<metadata><name>BeamNG.Earth drive</name><time>{iso(time.time())}</time></metadata>',
               '<trk><name>BeamNG.Earth drive</name>']
        for seg in self.segments():
            out.append("<trkseg>")
            for la, lo, t, v, _ in seg:
                out.append(f'<trkpt lat="{la:.7f}" lon="{lo:.7f}"><time>{iso(t)}</time></trkpt>')
            out.append("</trkseg>")
        out.append("</trk></gpx>")
        return "\n".join(out)

    def kml(self):
        s = self.summary()
        desc = (f"Distance {s['trip']['dist_m'] / 1000:.1f} km, top speed {s['trip']['max_ms'] * 3.6:.0f} km/h. "
                "Recorded with BeamNG.Earth Live GPS by Leopri2008.")
        out = ['<?xml version="1.0" encoding="UTF-8"?>',
               '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
               '<name>BeamNG.Earth drive</name>',
               f'<description>{escape(desc)}</description>',
               '<Style id="route"><LineStyle><color>ff1a7aff</color><width>4</width></LineStyle></Style>']
        for i, seg in enumerate(self.segments(), 1):
            coords = " ".join(f"{lo:.7f},{la:.7f},0" for la, lo, t, v, _ in seg)
            out.append(f'<Placemark><name>Part {i} ({iso(seg[0][2])})</name><styleUrl>#route</styleUrl>'
                       f'<LineString><tessellate>1</tessellate><coordinates>{coords}</coordinates></LineString>'
                       '</Placemark>')
        out.append("</Document></kml>")
        return "\n".join(out)


# ===================================================================== fog of the world
class Fog:
    """Explored map cells. A cell is explored only when REAL driving (the same check as the odometer) passes through it:
    teleports and glitches reveal nothing. Cells are stored as x * FOG_N + y of a zoom-20 map tile."""

    def __init__(self):
        self.lock = threading.Lock()
        self.cells = set()
        self.order = []            # the same cells in the order they were added (map page syncs incrementally)
        self.epoch = 1             # changes whenever cells are removed or restored: pages reload everything
        self.area_m2 = 0.0
        self.prev = None           # last revealed position, to fill the gap between two poses
        self.dirty = False
        self.saved_at = 0.0
        self._area = {}
        self._load()

    @staticmethod
    def _tile(lat, lon):
        lat = max(-85.0511, min(85.0511, lat))
        return ((lon + 180.0) / 360.0 * FOG_N,
                (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * FOG_N)

    def _cell_area(self, y):
        a = self._area.get(y)
        if a is None:
            lat = math.atan(math.sinh(math.pi * (1 - 2 * (y + 0.5) / FOG_N)))
            a = self._area[y] = (40075016.686 * math.cos(lat) / FOG_N) ** 2
        return a

    def _add(self, k):
        if k not in self.cells:
            self.cells.add(k)
            self.order.append(k)
            self.area_m2 += self._cell_area(k % FOG_N)

    def _load(self):
        try:
            with open(FOG_FILE, encoding="utf-8") as f:
                data = json.load(f)
            z = data.get("z")
            if not isinstance(z, int) or not 1 <= z <= FOG_Z:
                z = FOG_Z
            f = 1 << (FOG_Z - z)               # cells of an earlier build (coarser grid): each becomes f x f of today's
            for k in data.get("cells", []):
                if isinstance(k, int) and 0 <= k < (FOG_N // f) ** 2:
                    x, y = divmod(k, FOG_N // f)
                    for dx in range(f):
                        for dy in range(f):
                            self._add((x * f + dx) * FOG_N + y * f + dy)
        except (OSError, ValueError, AttributeError, TypeError):
            pass

    def save(self, force=False):
        now = time.time()
        if not (self.dirty or force) or (not force and now - self.saved_at < 10.0):
            return
        with self.lock:
            data = {"z": FOG_Z, "cells": list(self.order)}
            self.dirty = False
        try:
            tmp = FOG_FILE + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, separators=(",", ":"))
            os.replace(tmp, FOG_FILE)
            self.saved_at = now
        except OSError:
            self.dirty = True

    def cut(self):
        """A teleport or a long pause: the next reveal must not be joined to the previous place."""
        self.prev = None

    def reveal(self, lat, lon):
        with self.lock:
            pts = [(lat, lon)]
            if self.prev:
                d = dist_m(self.prev[0], self.prev[1], lat, lon)
                if d <= 300.0:             # fast driving: fill in between two poses, steps of at most half a cell
                    n = int(d / (0.5 * math.sqrt(self._cell_area(int(self._tile(*self.prev)[1]))))) + 1
                    pts = [(self.prev[0] + (lat - self.prev[0]) * i / n,
                            self.prev[1] + (lon - self.prev[1]) * i / n) for i in range(1, n + 1)]
            self.prev = (lat, lon)
            before = len(self.order)
            for la, lo in pts:
                x, y = self._tile(la, lo)
                r = FOG_REACH_M / math.sqrt(self._cell_area(int(y)))          # in cells
                for i in range(math.floor(x - r), math.floor(x + r) + 1):
                    for j in range(math.floor(y - r), math.floor(y + r) + 1):
                        if 0 <= j < FOG_N:
                            self._add((i % FOG_N) * FOG_N + j)
            if len(self.order) != before:
                self.dirty = True

    def edit(self, remove, add):
        """Map page edit mode: remove wrongly explored cells (or restore them with undo). Returns the new epoch."""
        with self.lock:
            for k in remove:
                if k in self.cells:
                    self.cells.discard(k)
                    self.area_m2 -= self._cell_area(k % FOG_N)
            self.order = [k for k in self.order if k in self.cells]
            for k in add:
                self._add(k)
            self.epoch += 1
            self.dirty = True
        self.save(force=True)
        return self.epoch

    def reset(self):
        with self.lock:
            self.cells.clear()
            self.order = []
            self.area_m2 = 0.0
            self.prev = None
            self.epoch += 1
            self.dirty = True
        self.save(force=True)

    def since(self, epoch, n):
        with self.lock:
            full = epoch != self.epoch or n < 0 or n > len(self.order)
            cells = list(self.order) if full else self.order[n:]
            return {"z": FOG_Z, "epoch": self.epoch, "full": full, "n": len(self.order), "cells": cells}

    def summary(self):
        return {"cells": len(self.order), "km2": round(max(self.area_m2, 0.0) / 1e6, 3), "epoch": self.epoch}


fog = None   # created in main()


def full_summary():
    with rec.lock:
        info = rec.summary()
    info["fog"] = fog.summary()
    if update_info:
        info["update"] = update_info
    return info


def check_update():
    """One quiet look at the latest GitHub release. Never blocks and never fails loudly (offline, rate limit, ...)."""
    try:
        req = urllib.request.Request(f"https://api.github.com/repos/{REPO}/releases/latest",
                                     headers={"User-Agent": "BNE-LiveGPS", "Accept": "application/vnd.github+json"})
        with urllib.request.urlopen(req, timeout=4) as r:
            j = json.loads(r.read(200000).decode("utf-8", "replace"))
        tag = re.sub(r"[^0-9A-Za-z.\-]", "", str(j.get("tag_name") or ""))[:20]
        ver = lambda t: tuple(int(p) for p in re.findall(r"\d+", t)[:3])
        if tag and ver(tag) > ver(VERSION):
            url = str(j.get("html_url") or "")
            if not url.startswith("https://github.com/"):
                url = f"https://github.com/{REPO}/releases/latest"
            update_info.update(version=tag.lstrip("vV"), url=url)
            print(f"\n  UPDATE AVAILABLE: v{update_info['version']} (you have v{VERSION})  {url}", flush=True)
    except Exception:
        pass


rec = None   # created in main()


def handle_pose(msg):
    global DEBUG
    DEBUG = False
    now = time.time()
    result = rec.feed(msg.get("lat"), msg.get("lon"), msg.get("speed"), now)
    if result in ("pending", "drop"):
        return                                   # not believed (yet): the live position stays where it was
    vx, vy = msg.get("vx") or 0.0, msg.get("vy") or 0.0     # m/s in the local frame: x = east, y = north
    if result == "jump":
        fog.cut()                                # a teleport reveals nothing, and is not joined to the old place
    else:
        if result == "resume":
            fog.cut()
        fog.reveal(msg["lat"], msg["lon"])
    with rec.lock:
        speed = rec.last["v"]          # the checked speed (spikes replaced by the last good value)
    info = full_summary()
    if result == "jump":
        vx = vy = 0.0
    pos = {
        "lat": msg.get("lat"),
        "lon": msg.get("lon"),
        "speed_kmh": round(speed * 3.6, 1),
        "speed_ms": speed,
        "ve": vx,
        "vn": vy,
        "bearing": (math.degrees(math.atan2(vx, vy)) + 360.0) % 360.0,
        "z_local": msg.get("z"),
        "anchorId": msg.get("anchorId"),
        "t": now,
        "event": result,
    }
    pos.update(info)
    with lock:
        latest.clear()
        latest.update(pos)
    rec.save()
    fog.save()
    try:
        tmp = OUT_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(pos, f)
        os.replace(tmp, OUT_FILE)
    except OSError:
        pass
    print(f"\r  LIVE  {pos['lat']:.6f}, {pos['lon']:.6f}  {pos['speed_kmh']:6.1f} km/h  |  "
          f"trip {info['trip']['dist_m'] / 1000:7.2f} km  |  total {info['odo_m'] / 1000:9.2f} km   ",
          end="", flush=True)


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
                if b'"pose"' not in payload:  # cheap skip of the big non-pose messages
                    if DEBUG:
                        seen["other"] = seen.get("other", 0) + 1
                    continue
                try:
                    msg = json.loads(payload)   # field order from Lua is random -> always parse fully
                except ValueError:
                    continue
                t = msg.get("type") if isinstance(msg, dict) else None
                seen[t] = seen.get(t, 0) + 1
                if t == "pose":
                    try:
                        handle_pose(msg)
                    except Exception as e:      # never let a bad pose break the relay
                        print(f"\n  [warn] pose skipped: {e}", flush=True)
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
    def _q(self):
        q = {}
        if "?" in self.path:
            for part in self.path.split("?", 1)[1].split("&"):
                k, _, v = part.partition("=")
                q[k] = v
        return q

    def do_GET(self):
        path = self.path.split("?", 1)[0].rstrip("/") or "/"
        if path in ("/", "/pos"):
            with lock:
                data = dict(latest)
            if "t" in data:
                data["age"] = round(time.time() - data["t"], 3)   # seconds since the last pose
            else:
                data.update(full_summary())                        # stats even before the first pose
            self._send(200, "application/json", json.dumps(data).encode())
        elif path == "/track":
            q = self._q()
            try:
                since = int(q.get("since", "0"))
            except ValueError:
                since = 0
            self._send(200, "application/json",
                       json.dumps(rec.track_since(since, q.get("id", "")), separators=(",", ":")).encode())
        elif path == "/stats":
            self._send(200, "application/json", json.dumps(full_summary()).encode())
        elif path == "/fog":
            q = self._q()
            try:
                epoch, n = int(q.get("epoch", "0")), int(q.get("n", "0"))
            except ValueError:
                epoch, n = 0, 0
            self._send(200, "application/json", json.dumps(fog.since(epoch, n), separators=(",", ":")).encode())
        elif path in ("/export.gpx", "/export.kml"):
            kind = path[-3:]
            body = (rec.gpx() if kind == "gpx" else rec.kml()).encode("utf-8")
            name = "beamng-earth-drive-" + datetime.now().strftime("%Y%m%d-%H%M") + "." + kind
            ctype = "application/gpx+xml" if kind == "gpx" else "application/vnd.google-earth.kml+xml"
            self._send(200, ctype, body, {"Content-Disposition": f'attachment; filename="{name}"'})
        elif path == "/map":
            self._send(200, "text/html; charset=utf-8", MAP_HTML.encode("utf-8"))
        else:
            self._send(404, "text/plain", b"Not found. Try /map for the live map.")

    def do_POST(self):
        path = self.path.split("?", 1)[0].rstrip("/")
        if path == "/reset":
            rec.reset_trip()
            print("\n  Trip and route reset (odometer kept).", flush=True)
            info = full_summary()
            with lock:
                latest.update(info)
            self._send(200, "application/json", json.dumps(info).encode())
        elif path == "/fog/edit":
            try:
                body = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length", "0")), 2_000_000)))
                lists = []
                for key in ("remove", "add"):
                    ks = body.get(key, [])
                    if not isinstance(ks, list) or len(ks) > FOG_MAX_EDIT or not all(
                            type(k) is int and 0 <= k < FOG_N * FOG_N for k in ks):
                        raise ValueError(key)
                    lists.append(ks)
            except (ValueError, TypeError, AttributeError):
                self._send(400, "text/plain", b"Bad request")
                return
            self._send(200, "application/json", json.dumps({"epoch": fog.edit(*lists)}).encode())
        elif path == "/fog/reset":
            fog.reset()
            print("\n  Fog of the world reset.", flush=True)
            self._send(200, "application/json", json.dumps({"epoch": fog.epoch}).encode())
        else:
            self._send(404, "text/plain", b"Not found")

    def _send(self, code, ctype, body, headers=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        try:
            self.wfile.write(body)
        except OSError:
            pass

    def log_message(self, *a):
        pass


# Live map page served at /map (MapLibre GL JS + OpenFreeMap tiles, loaded by the browser).
MAP_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover, user-scalable=no">
<meta name="theme-color" content="#0d1015">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="BNE Live Map">
<title>BNE Live Map</title>
<link rel="stylesheet" href="https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.css">
<script src="https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.js"></script>
<style>
/* BeamNG.Earth Live GPS - live map. Part of BeamNG.Earth Live GPS by Leopri2008. See LICENSE. */
:root {
  --bg: #0d1015;
  --glass: rgba(16, 20, 27, 0.78);
  --glass-strong: rgba(16, 20, 27, 0.94);
  --line: rgba(255, 255, 255, 0.09);
  --text: #e8ecf1;
  --muted: #8b95a3;
  --accent: #ff7a1a;
  --accent-ink: #1a0e05;
  --live: #3ddc84;
  --warn: #f5b841;
  --bad: #ff5c5c;
  --radius: 18px;
  --gap: clamp(8px, 1.6vmin, 14px);
  --edge-t: max(var(--gap), env(safe-area-inset-top));
  --edge-r: max(var(--gap), env(safe-area-inset-right));
  --edge-b: max(var(--gap), env(safe-area-inset-bottom));
  --edge-l: max(var(--gap), env(safe-area-inset-left));
  --ctl: 46px;
  --shadow: 0 8px 28px rgba(0, 0, 0, 0.35);
  --font: system-ui, -apple-system, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
}
* { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
html, body { margin: 0; height: 100%; background: var(--bg); color: var(--text); font-family: var(--font); overflow: hidden; }
body { position: fixed; inset: 0; }
button { font: inherit; color: inherit; background: none; border: 0; padding: 0; cursor: pointer; }
#map { position: absolute; inset: 0; background: var(--bg); z-index: 1; }

.glass {
  background: var(--glass);
  border: 1px solid var(--line);
  box-shadow: var(--shadow);
  -webkit-backdrop-filter: blur(14px) saturate(140%);
  backdrop-filter: blur(14px) saturate(140%);
}
.hidden { display: none !important; }

/* ---------- top bar ---------- */
#top {
  position: absolute; top: var(--edge-t); left: var(--edge-l); right: calc(var(--edge-r) + var(--ctl) + var(--gap));
  display: flex; gap: var(--gap); align-items: center; pointer-events: none; z-index: 3;
}
.pill {
  pointer-events: auto; height: 38px; padding: 0 14px; border-radius: 999px;
  display: inline-flex; align-items: center; gap: 9px; white-space: nowrap; font-size: 13px;
}
#status { font-weight: 650; letter-spacing: 0.1em; text-transform: uppercase; font-size: 12px; }
#status .dot { width: 9px; height: 9px; border-radius: 50%; background: var(--muted); flex: none; }
#status[data-s="live"] .dot { background: var(--live); box-shadow: 0 0 0 0 rgba(61, 220, 132, .6); animation: pulse 2s infinite; }
#status[data-s="live"] { color: var(--live); }
#status[data-s="stale"], #status[data-s="waiting"], #status[data-s="connecting"] { color: var(--warn); }
#status[data-s="stale"] .dot, #status[data-s="waiting"] .dot, #status[data-s="connecting"] .dot { background: var(--warn); }
#status[data-s="offline"] { color: var(--bad); }
#status[data-s="offline"] .dot { background: var(--bad); }
@keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(61,220,132,.55); } 70% { box-shadow: 0 0 0 9px rgba(61,220,132,0); } 100% { box-shadow: 0 0 0 0 rgba(61,220,132,0); } }
#brand { color: var(--muted); font-weight: 600; letter-spacing: 0; text-transform: none; font-size: 12px; padding-left: 4px; border-left: 1px solid var(--line); margin-left: 2px; padding-left: 10px; }
#coords { font-variant-numeric: tabular-nums; font-family: ui-monospace, "SF Mono", Menlo, Consolas, monospace; font-size: 12.5px; color: var(--text); min-width: 0; overflow: hidden; text-overflow: ellipsis; }
#coords:hover { border-color: rgba(255,255,255,.2); }

/* ---------- right controls ---------- */
#controls {
  position: absolute; top: var(--edge-t); right: var(--edge-r); z-index: 3;
  display: flex; flex-direction: column; gap: 8px; border-radius: var(--radius); padding: 5px;
}
.ctl {
  width: var(--ctl); height: var(--ctl); border-radius: 13px; display: grid; place-items: center;
  color: var(--text); transition: background .15s, color .15s;
}
.ctl:hover { background: rgba(255,255,255,.07); }
.ctl:active { transform: scale(.96); }
.ctl.on { background: var(--accent); color: var(--accent-ink); }
.ctl svg { width: 22px; height: 22px; }
.ctl .t3d { font-weight: 800; font-size: 14px; letter-spacing: .02em; }
.sep { height: 1px; background: var(--line); margin: 0 6px; }

#recenter {
  position: absolute; left: 50%; transform: translateX(-50%); z-index: 3;
  bottom: calc(var(--hud-h, 120px) + var(--edge-b) + var(--gap));
  height: 44px; padding: 0 18px; border-radius: 999px; display: inline-flex; align-items: center; gap: 8px;
  background: var(--accent); color: var(--accent-ink); font-weight: 700; font-size: 14px; box-shadow: var(--shadow);
}
#recenter svg { width: 18px; height: 18px; }

/* ---------- bottom HUD ---------- */
#hud {
  position: absolute; left: var(--edge-l); right: var(--edge-r); bottom: calc(var(--edge-b) + 14px); z-index: 2;
  display: flex; gap: var(--gap); align-items: flex-end; pointer-events: none;
}
#hud > * { pointer-events: auto; }
.card { border-radius: var(--radius); }

#speedCard { padding: 12px 18px 12px 16px; display: flex; align-items: center; gap: 16px; user-select: none; flex: none; }
.spd { display: flex; flex-direction: column; align-items: flex-start; line-height: 1; cursor: pointer; }
#spd { font-size: clamp(40px, 9vmin, 76px); font-weight: 750; font-variant-numeric: tabular-nums; letter-spacing: -0.03em; min-width: 2.2ch; }
#unit { font-size: 12px; color: var(--muted); font-weight: 650; letter-spacing: .12em; text-transform: uppercase; margin-top: 4px; }
.compass { display: flex; flex-direction: column; align-items: center; gap: 6px; }
.dial { width: clamp(42px, 8vmin, 58px); height: clamp(42px, 8vmin, 58px); border-radius: 50%; border: 1.5px solid var(--line); position: relative; display: grid; place-items: center; }
.dial .n { position: absolute; top: 2px; font-size: 9px; font-weight: 800; color: var(--accent); }
#needle { width: 52%; height: 52%; transition: transform .25s linear; }
#hdg { font-size: 12px; color: var(--muted); font-variant-numeric: tabular-nums; font-weight: 600; white-space: nowrap; }

#stats { flex: 1; min-width: 0; max-width: 680px; padding: 12px 8px 12px 16px; display: flex; align-items: center; gap: 4px; }
.stat-grid { flex: 1; display: grid; grid-template-columns: repeat(5, minmax(0, 1fr)); gap: 4px 12px; }
.stat.odo { border-left: 1px solid var(--line); padding-left: 12px; }
.stat { min-width: 0; }
.stat b { display: block; font-size: clamp(16px, 2.6vmin, 22px); font-weight: 700; font-variant-numeric: tabular-nums; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
.stat span { font-size: 10.5px; color: var(--muted); text-transform: uppercase; letter-spacing: .1em; font-weight: 650; }
.icon-btn { width: 40px; height: 40px; border-radius: 12px; display: grid; place-items: center; color: var(--muted); flex: none; }
.icon-btn:hover { background: rgba(255,255,255,.07); color: var(--text); }
.icon-btn svg { width: 19px; height: 19px; }

#attrib {
  position: absolute; right: var(--edge-r); bottom: max(2px, env(safe-area-inset-bottom)); z-index: 2;
  font-size: 10px; color: rgba(232, 236, 241, .7); text-shadow: 0 1px 2px rgba(0,0,0,.8); white-space: nowrap;
}
#attrib a { color: inherit; text-decoration: none; }
#attrib a:hover { text-decoration: underline; }

/* ---------- marker ---------- */
.car { width: 46px; height: 46px; display: grid; place-items: center; pointer-events: none; }
.car .halo { position: absolute; inset: 4px; border-radius: 50%; background: rgba(255,122,26,.18); border: 1.5px solid rgba(255,122,26,.45); }
.car svg { width: 30px; height: 30px; position: relative; filter: drop-shadow(0 2px 4px rgba(0,0,0,.5)); }
.car.stale .halo { background: rgba(139,149,163,.18); border-color: rgba(139,149,163,.45); }
.car.stale path { fill: #8b95a3; }

/* ---------- settings sheet ---------- */
#scrim { position: absolute; inset: 0; z-index: 4; background: rgba(0,0,0,.35); }
#sheet {
  position: absolute; z-index: 5; left: 0; right: 0; bottom: 0;
  max-height: min(78dvh, 640px); overflow: auto; overscroll-behavior: contain;
  background: var(--glass-strong); border-top: 1px solid var(--line); border-radius: 22px 22px 0 0;
  padding: 10px max(18px, env(safe-area-inset-right)) max(18px, env(safe-area-inset-bottom)) max(18px, env(safe-area-inset-left));
  box-shadow: 0 -10px 40px rgba(0,0,0,.45);
  -webkit-backdrop-filter: blur(18px); backdrop-filter: blur(18px);
  animation: up .22s ease-out;
}
@keyframes up { from { transform: translateY(24px); opacity: 0; } to { transform: none; opacity: 1; } }
.grab { width: 40px; height: 4px; border-radius: 4px; background: rgba(255,255,255,.18); margin: 2px auto 10px; }
.sheet-head { display: flex; align-items: center; justify-content: space-between; margin-bottom: 4px; }
.sheet-head h2 { margin: 0; font-size: 18px; font-weight: 700; }
.sec { margin-top: 18px; }
.sec h3 { margin: 0 0 10px; font-size: 11px; letter-spacing: .14em; text-transform: uppercase; color: var(--muted); font-weight: 700; }
.styles { display: grid; grid-template-columns: repeat(auto-fill, minmax(92px, 1fr)); gap: 10px; }
.sty { border-radius: 14px; border: 1.5px solid var(--line); padding: 6px; text-align: left; transition: border-color .15s; }
.sty:hover { border-color: rgba(255,255,255,.25); }
.sty.on { border-color: var(--accent); }
.sty i { display: block; height: 46px; border-radius: 9px; margin-bottom: 6px; }
.sty span { font-size: 13px; font-weight: 600; padding-left: 2px; }
.seg { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; background: rgba(255,255,255,.05); border-radius: 14px; padding: 4px; }
.seg button { height: 40px; border-radius: 10px; font-size: 13px; font-weight: 650; color: var(--muted); }
.seg button.on { background: var(--accent); color: var(--accent-ink); }
.row { display: flex; align-items: center; justify-content: space-between; gap: 12px; padding: 12px 2px; border-bottom: 1px solid var(--line); }
.row:last-child { border-bottom: 0; }
.row label { font-size: 15px; }
.row small { display: block; color: var(--muted); font-size: 12px; margin-top: 2px; }
.switch { width: 50px; height: 30px; border-radius: 30px; background: rgba(255,255,255,.14); position: relative; flex: none; transition: background .15s; }
.switch::after { content: ""; position: absolute; top: 3px; left: 3px; width: 24px; height: 24px; border-radius: 50%; background: #fff; transition: transform .18s; }
.switch[aria-checked="true"] { background: var(--accent); }
.switch[aria-checked="true"]::after { transform: translateX(20px); }
.btn { height: 44px; padding: 0 16px; border-radius: 12px; font-weight: 650; font-size: 14px; border: 1.5px solid var(--line); }
.btn:hover { border-color: rgba(255,255,255,.25); }
.btn.warn { color: var(--bad); border-color: rgba(255,92,92,.35); }
a.btn { display: inline-flex; align-items: center; text-decoration: none; color: var(--text); }
.btn.armed, .icon-btn.armed { background: var(--bad); color: #fff; border-color: var(--bad); }
.odo-box { border: 1px solid var(--line); border-radius: 14px; padding: 12px 14px; margin-bottom: 10px; background: rgba(255,255,255,.03); }
.odo-box span { display: block; font-size: 12px; color: var(--muted); }
.odo-box b { display: block; font-size: 28px; font-weight: 750; font-variant-numeric: tabular-nums; letter-spacing: -0.02em; margin: 2px 0; }
.odo-box small { display: block; font-size: 11.5px; color: var(--muted); }
.btns { display: flex; flex-wrap: wrap; gap: 8px; }
.note { font-size: 12.5px; color: var(--muted); line-height: 1.5; margin: 8px 2px 0; }
.note code { color: var(--text); font-size: 12px; background: rgba(255,255,255,.06); padding: 1px 6px; border-radius: 6px; word-break: break-all; }
kbd { font-family: inherit; font-size: 11px; border: 1px solid var(--line); border-bottom-width: 2px; border-radius: 5px; padding: 0 5px; color: var(--text); }

/* ---------- waiting overlay ---------- */
#waiting { position: absolute; inset: 0; z-index: 2; display: grid; place-items: center; padding: 24px; pointer-events: none; }
#waiting .card { pointer-events: auto; max-width: 380px; padding: 22px 22px 18px; text-align: center; }
#waiting h1 { margin: 6px 0 6px; font-size: 19px; }
#waiting p { margin: 0; color: var(--muted); font-size: 14px; line-height: 1.5; }
.spinner { width: 34px; height: 34px; margin: 0 auto 4px; border-radius: 50%; border: 3px solid rgba(255,255,255,.12); border-top-color: var(--accent); animation: spin 1s linear infinite; }
@keyframes spin { to { transform: rotate(360deg); } }

#awakeHint {
  position: absolute; z-index: 4; left: 50%; top: calc(var(--edge-t) + 50px); transform: translateX(-50%);
  display: inline-flex; align-items: center; gap: 10px; max-width: calc(100vw - 2 * var(--gap) - var(--ctl) - 24px);
  padding: 10px 16px 10px 12px; border-radius: 999px; font-size: 13.5px; white-space: nowrap; font-weight: 650; line-height: 1.25;
  color: var(--text); border-color: rgba(255,122,26,.45); animation: hintIn .3s ease-out, hintPulse 2.4s ease-in-out .3s infinite;
}
#awakeHint svg { width: 22px; height: 22px; flex: none; color: var(--accent); }
@keyframes hintIn { from { opacity: 0; transform: translateX(-50%) translateY(-6px); } to { opacity: 1; transform: translateX(-50%); } }
@keyframes hintPulse { 0%,100% { box-shadow: var(--shadow), 0 0 0 0 rgba(255,122,26,.35); } 50% { box-shadow: var(--shadow), 0 0 0 7px rgba(255,122,26,0); } }
body.obs #awakeHint { display: none !important; }
@media (max-width: 520px) {
  #awakeHint { left: var(--edge-l); transform: none; white-space: normal; text-align: left;
    max-width: calc(100vw - var(--edge-l) - var(--edge-r) - var(--ctl) - 26px); animation: hintPulse 2.4s ease-in-out infinite; }
}
#toast {
  position: absolute; z-index: 6; left: 50%; top: calc(var(--edge-t) + 104px); transform: translateX(-50%);
  padding: 10px 16px; border-radius: 999px; font-size: 13.5px; font-weight: 600; pointer-events: none;
  opacity: 0; transition: opacity .2s, transform .2s;
}
#toast.show { opacity: 1; transform: translateX(-50%) translateY(4px); }

/* ---------- desktop / tablet ---------- */
@media (min-width: 760px) {
  #sheet {
    left: auto; right: calc(var(--edge-r) + var(--ctl) + 10px + var(--gap)); top: var(--edge-t); bottom: auto;
    width: 360px; max-height: calc(100dvh - var(--edge-t) - var(--edge-b) - 40px);
    border: 1px solid var(--line); border-radius: 20px; padding: 16px 18px 18px;
    animation: pop .18s ease-out;
  }
  #scrim { background: transparent; }
  .grab { display: none; }
  @keyframes pop { from { transform: translateY(-6px); opacity: 0; } to { transform: none; opacity: 1; } }
}
@media (min-width: 1100px) {
  :root { --ctl: 48px; }
  #speedCard { padding: 16px 22px 16px 20px; }
}

/* ---------- small phones ---------- */
@media (max-width: 520px) {
  #brand { display: none; }
  #hud { flex-direction: column; align-items: stretch; }
  #speedCard { align-self: flex-start; }
  #stats { max-width: none; padding: 10px 6px 10px 14px; }
  .stat b { font-size: 16px; }
  .stat span { font-size: 9.5px; }
  .stat-grid { grid-template-columns: repeat(3, minmax(0, 1fr)); }
  .stat.odo { border-left: 0; padding-left: 0; }
}
@media (max-width: 380px) {
  #coords { display: none; }
  .stat-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
}
/* landscape phones: everything in one compact row */
@media (max-height: 520px) and (orientation: landscape) {
  :root { --ctl: 40px; }
  #controls { gap: 4px; padding: 4px; }
  .ctl { border-radius: 11px; }
  .ctl svg { width: 19px; height: 19px; }
  #hud { flex-direction: row; align-items: flex-end; right: calc(var(--edge-r) + var(--ctl) + 10px + var(--gap)); }
  #speedCard { padding: 8px 14px; gap: 12px; }
  #spd { font-size: 40px; }
  .dial { width: 40px; height: 40px; }
  #stats { padding: 8px 6px 8px 12px; }
  .stat b { font-size: 15px; }
  #sheet { max-height: 92dvh; }
}

/* ---------- OBS / overlay mode ---------- */
body.obs #top, body.obs #controls, body.obs #recenter, body.obs #waiting, body.obs #stats { display: none !important; }
body.obs.obs-stats #stats { display: flex !important; }

/* ---------- fog of the world, update notice ---------- */
#fog { position: absolute; left: 0; top: 0; width: 100%; height: 100%; pointer-events: none; }
#fogBar {
  position: absolute; z-index: 4; left: 50%; top: calc(var(--edge-t) + 50px); transform: translateX(-50%);
  display: none; align-items: center; justify-content: center; flex-wrap: wrap; gap: 8px;
  padding: 8px 10px; border-radius: 16px; max-width: calc(100vw - 2 * var(--gap));
}
#fogBar b { font-size: 13px; padding: 0 4px; }
#fogBar .seg { grid-template-columns: repeat(3, 52px); padding: 3px; gap: 3px; }
#fogBar .seg button { height: 34px; }
#fogBar .btn { height: 40px; padding: 0 14px; }
#fogBar .btn.go { background: var(--accent); color: var(--accent-ink); border-color: var(--accent); }
body.fogedit #fogBar { display: flex; }
body.fogedit #awakeHint, body.fogedit #updPill, body.fogedit #recenter { display: none !important; }
#updPill {
  position: absolute; z-index: 4; left: 50%; top: calc(var(--edge-t) + 50px); transform: translateX(-50%);
  display: flex; align-items: center; gap: 4px; padding: 4px 4px 4px 14px; border-radius: 999px;
  font-size: 13px; font-weight: 650; white-space: nowrap; border-color: rgba(61,220,132,.45);
}
#updPill a { color: var(--live); text-decoration: none; }
#updPill button { width: 30px; height: 30px; border-radius: 50%; color: var(--muted); font-size: 18px; line-height: 1; }
body.upd #awakeHint { top: calc(var(--edge-t) + 96px); }
body.obs #updPill, body.obs #fogBar { display: none !important; }
</style>
</head>
<body>
<div id="map" aria-label="Live map"></div>

<header id="top">
  <div id="status" class="pill glass" data-s="connecting" role="status"><span class="dot"></span><span id="statusText">Connecting</span><span id="brand">BeamNG.Earth Live GPS</span></div>
  <button id="coords" class="pill glass" title="Copy coordinates">—</button>
</header>

<nav id="controls" class="glass" aria-label="Map controls">
  <button class="ctl" id="btnCam" title="Camera mode (C)" aria-label="Camera mode"></button>
  <button class="ctl" id="btn3d" title="3D buildings (B)" aria-label="3D buildings"><span class="t3d">3D</span></button>
  <button class="ctl" id="btnStyle" title="Map style (S)" aria-label="Map style">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"><path d="M12 3 2.5 8 12 13l9.5-5L12 3Z"/><path d="m2.5 12.5 9.5 5 9.5-5"/><path d="m2.5 16.5 9.5 5 9.5-5" opacity=".55"/></svg>
  </button>
  <div class="sep"></div>
  <button class="ctl" id="btnFull" title="Fullscreen (F)" aria-label="Fullscreen">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><path d="M4 9V4h5M20 9V4h-5M4 15v5h5M20 15v5h-5"/></svg>
  </button>
  <button class="ctl" id="btnSettings" title="Settings" aria-label="Settings">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round"><path d="M4 7h10M18 7h2M4 17h4M12 17h8"/><circle cx="16" cy="7" r="2"/><circle cx="10" cy="17" r="2"/></svg>
  </button>
</nav>

<button id="recenter" class="hidden">
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"><circle cx="12" cy="12" r="3.2"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/></svg>
  Recenter
</button>

<section id="hud">
  <div id="speedCard" class="card glass">
    <div class="spd" id="spdBox" title="Tap to switch km/h / mph"><span id="spd">0</span><span id="unit">km/h</span></div>
    <div class="compass">
      <div class="dial"><span class="n">N</span>
        <svg id="needle" viewBox="0 0 24 24"><path d="M12 2 17 20 12 16.5 7 20Z" fill="#ff7a1a"/></svg>
      </div>
      <span id="hdg">— —</span>
    </div>
  </div>
  <div id="stats" class="card glass">
    <div class="stat-grid">
      <div class="stat"><b id="stDist">0.0 km</b><span>Distance</span></div>
      <div class="stat"><b id="stTime">0:00</b><span>Drive time</span></div>
      <div class="stat"><b id="stAvg">0</b><span>Avg</span></div>
      <div class="stat"><b id="stMax">0</b><span>Max</span></div>
      <div class="stat odo"><b id="stOdo">0 km</b><span>Total</span></div>
    </div>
    <button class="icon-btn" id="btnReset" title="Reset trip" aria-label="Reset trip">
      <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M3 12a9 9 0 1 0 3-6.7"/><path d="M3 4v5h5"/></svg>
    </button>
  </div>
</section>

<div id="attrib"><a href="https://openfreemap.org" target="_blank" rel="noopener">OpenFreeMap</a> © <a href="https://www.openmaptiles.org/" target="_blank" rel="noopener">OpenMapTiles</a> Data from <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap</a></div>

<div id="waiting">
  <div class="card glass">
    <div class="spinner"></div>
    <h1 id="waitTitle">Connecting…</h1>
    <p id="waitText">Looking for the BeamNG.Earth Live GPS script on your PC.</p>
  </div>
</div>

<div id="scrim" class="hidden"></div>
<aside id="sheet" class="hidden" role="dialog" aria-label="Settings">
  <div class="grab"></div>
  <div class="sheet-head"><h2>Map settings</h2>
    <button class="icon-btn" id="btnClose" aria-label="Close"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M6 6l12 12M18 6 6 18"/></svg></button>
  </div>

  <div class="sec"><h3>Map style</h3><div class="styles" id="styleList"></div></div>

  <div class="sec"><h3>Camera</h3>
    <div class="seg" id="camSeg">
      <button data-cam="north">North up</button>
      <button data-cam="heading">Heading up</button>
      <button data-cam="chase">Chase 3D</button>
    </div>
  </div>

  <div class="sec"><h3>Display</h3>
    <div class="row"><label>3D view<small>Tilted map with 3D buildings when zoomed in</small></label><button class="switch" data-opt="buildings" role="switch"></button></div>
    <div class="row"><label>Auto zoom<small>Zooms out the faster you drive</small></label><button class="switch" data-opt="autoZoom" role="switch"></button></div>
    <div class="row"><label>Driven route<small>Draws your path on the map</small></label><button class="switch" data-opt="trail" role="switch"></button></div>
    <div class="row"><label>Trip stats</label><button class="switch" data-opt="stats" role="switch"></button></div>
    <div class="row"><label>Keep screen on<small>Stops the phone or tablet from going to sleep</small></label><button class="switch" data-opt="awake" role="switch"></button></div>
    <div class="row"><label>Units</label>
      <div class="seg" id="unitSeg" style="grid-template-columns:repeat(2,1fr);width:150px"><button data-unit="metric">km/h</button><button data-unit="imperial">mph</button></div>
    </div>
  </div>

  <div class="sec"><h3>Trip &amp; route</h3>
    <div class="odo-box"><span>Total driven with Live GPS</span><b id="odoBig">0 km</b>
      <small>Only real driving counts: teleports and glitches are never added.</small></div>
    <p class="note" id="tripInfo"></p>
    <div class="btns">
      <a class="btn" id="expGpx" href="/export.gpx" download>Export GPX</a>
      <a class="btn" id="expKml" href="/export.kml" download>Export KML</a>
      <button class="btn warn" id="btnReset2">Reset trip &amp; route</button>
    </div>
    <p class="note">The route and trip are recorded by the script on your PC, also while this page is closed, and kept until you reset them. GPX works in most GPS apps and Strava-like tools, KML opens in Google Earth.</p>
  </div>

  <div class="sec"><h3>Fog of the world</h3>
    <div class="row"><label>Fog<small>Places you have not driven through yet stay hidden in a mist</small></label><button class="switch" data-opt="fog" role="switch"></button></div>
    <p class="note" id="fogInfo"></p>
    <div class="btns">
      <button class="btn" id="btnFogEdit">Erase cells</button>
      <button class="btn warn" id="btnFogReset">Reset fog</button>
    </div>
    <p class="note">Every place you really drive through is revealed; teleports and glitches reveal nothing. Erase mode shows the grid and lets you remove cells that were revealed by mistake. Reset fog and Reset trip are separate.</p>
  </div>

  <div class="sec"><h3>Tips</h3>
    <p class="note">Open this page on any device in your Wi-Fi, including iPhone and iPad. For full screen on iPhone: Share → <b>Add to Home Screen</b>. The screen stays on while this page is open (setting below the map options). If it still turns off, tap the map once.</p>
    <p class="note">Stream overlay for OBS (browser source): <code id="obsUrl"></code></p>
    <p class="note" id="keys">Keys: <kbd>C</kbd> camera · <kbd>B</kbd> 3D · <kbd>S</kbd> style · <kbd>R</kbd> recenter · <kbd>F</kbd> fullscreen · <kbd>U</kbd> units</p>
  </div>
</aside>

<button id="awakeHint" class="glass hidden" aria-live="polite">
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round"><path d="M9 11V5.5a1.5 1.5 0 0 1 3 0V10"/><path d="M12 10V8.5a1.5 1.5 0 0 1 3 0V11"/><path d="M15 11v-1a1.5 1.5 0 0 1 3 0v4.5a6 6 0 0 1-6 6h-.6a6 6 0 0 1-4.5-2.1L4.3 15a1.5 1.5 0 0 1 2.3-1.9L9 15.5V11"/></svg>
  <span>Tap once to keep screen on</span>
</button>
<div id="fogBar" class="glass">
  <b>Erase mode</b>
  <div class="seg" id="brushSeg"><button data-b="0">1 cell</button><button data-b="1">7×7</button><button data-b="2">17×17</button></div>
  <button class="btn" id="fogUndo">Undo</button>
  <button class="btn go" id="fogDone">Done</button>
</div>
<div id="updPill" class="glass hidden"><a id="updLink" target="_blank" rel="noopener">Update available</a><button id="updX" aria-label="Dismiss">×</button></div>
<div id="toast" class="glass"></div>

<script>
"use strict";
(() => {
  // ---------------------------------------------------------------- config
  const STYLES = [
    { id: "dark", name: "Dark", url: "https://tiles.openfreemap.org/styles/dark", dark: true, sw: "linear-gradient(135deg,#16261b,#1b2029 45%,#1d4057)" },
    { id: "liberty", name: "Liberty", url: "https://tiles.openfreemap.org/styles/liberty", dark: false, sw: "linear-gradient(135deg,#f2efe9,#cfe3c4 55%,#a6cbe6)" },
    { id: "bright", name: "Bright", url: "https://tiles.openfreemap.org/styles/bright", dark: false, sw: "linear-gradient(135deg,#f8f4f0,#f6d38b 55%,#9ec9f0)" },
    { id: "positron", name: "Positron", url: "https://tiles.openfreemap.org/styles/positron", dark: false, sw: "linear-gradient(135deg,#fafafa,#e6e6e6 55%,#cfd8dc)" },
    { id: "fiord", name: "Fiord", url: "https://tiles.openfreemap.org/styles/fiord", dark: true, sw: "linear-gradient(135deg,#45516e,#5e6f8f 55%,#2b3346)" },
  ];
  const CAMS = ["north", "heading", "chase"];
  const POLL_MS = 250;
  const STALE_S = 5;
  const SNAP_GAP_MS = 2500;           // longer without a new position (page in the background, ...): jump, don't glide
  const JOIN_MAX_M = 400;            // connect the route's last point to the car only if it's this close
  const BLD_DARK = "#5b616b", BLD_LIGHT = "#ddd6cb";   // neutral grey in dark styles
  const SRC_TRAIL = "bne-trail", LYR_TRAIL = "bne-trail-line", LYR_TRAIL_GLOW = "bne-trail-glow", LYR_BLD = "bne-3d-buildings";

  const params = new URLSearchParams(location.search);
  const OBS = params.get("obs") === "1";

  // ---------------------------------------------------------------- settings (per browser)
  const DEFAULTS = { style: "dark", cam: "chase", buildings: true, autoZoom: true, trail: true, stats: true, units: "metric", awake: true, fog: false };
  const store = {
    load() { try { return JSON.parse(localStorage.getItem("bne-map") || "{}"); } catch (e) { return {}; } },
    save(s) { try { localStorage.setItem("bne-map", JSON.stringify(s)); } catch (e) {} },
  };
  const S = Object.assign({}, DEFAULTS, OBS ? {} : store.load());
  // URL overrides (handy for OBS): ?style=liberty&cam=heading&units=imperial&buildings=0&trail=0&stats=1&fog=0
  for (const k of ["style", "cam", "units"]) if (params.get(k)) S[k] = params.get(k);
  for (const k of ["buildings", "autoZoom", "trail", "stats", "fog"]) if (params.has(k)) S[k] = params.get(k) !== "0";
  if (!STYLES.find(s => s.id === S.style)) S.style = "dark";
  if (!CAMS.includes(S.cam)) S.cam = "chase";
  const save = () => { if (!OBS) store.save(S); };

  if (OBS) { document.body.classList.add("obs"); if (params.get("stats") === "1") document.body.classList.add("obs-stats"); }

  // ---------------------------------------------------------------- helpers
  const $ = id => document.getElementById(id);
  const R = 6371008.8;
  const toRad = d => d * Math.PI / 180;
  function dist(a, b) {
    const dLat = toRad(b[1] - a[1]), dLon = toRad(b[0] - a[0]);
    const s = Math.sin(dLat / 2) ** 2 + Math.cos(toRad(a[1])) * Math.cos(toRad(b[1])) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.min(1, Math.sqrt(s)));
  }
  const angDiff = (a, b) => ((b - a + 540) % 360) - 180;
  const lerp = (a, b, t) => a + (b - a) * t;
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
  const compass = d => COMPASS[Math.round(((d % 360) + 360) % 360 / 45) % 8];
  const imperial = () => S.units === "imperial";
  const spdOut = ms => imperial() ? ms * 2.236936 : ms * 3.6;
  const spdUnit = () => imperial() ? "mph" : "km/h";
  const distOut = m => imperial() ? (m / 1609.344).toFixed(m < 160934 ? 1 : 0) + " mi" : (m / 1000).toFixed(m < 100000 ? 1 : 0) + " km";
  function fmtTime(s) {
    s = Math.floor(s); const h = Math.floor(s / 3600), m = Math.floor(s % 3600 / 60), x = s % 60;
    return h ? `${h}:${String(m).padStart(2, "0")}:${String(x).padStart(2, "0")}` : `${m}:${String(x).padStart(2, "0")}`;
  }
  let toastT = 0;
  function toast(msg) {
    const t = $("toast"); t.textContent = msg; t.classList.add("show");
    clearTimeout(toastT); toastT = setTimeout(() => t.classList.remove("show"), 1600);
  }

  // ---------------------------------------------------------------- live state
  const st = {
    status: "connecting", fails: 0,
    lastT: null,                 // server timestamp of the last sample
    sample: null,                // { lng, lat, ve, vn, speed, bearing, at (performance.now of the sample) }
    off: [0, 0], offAt: 0,       // smoothing offset (deg) when a new sample corrects the prediction
    disp: null,                  // displayed [lng, lat]
    bearing: 0, dispBearing: 0, speed: 0,
    stale: false,
    hasFix: false,
  };
  // trip stats, odometer and the route all come from the script on the PC (it records with this page closed too)
  const trip = { dist: 0, moving: 0, max: 0, odo: 0 };
  const trail = { id: null, n: 0, segs: [], segIds: [], dirty: false, loading: false, seg: null };

  async function loadTrack(full) {
    if (trail.loading) return;
    trail.loading = true;
    try {
      const since = full ? 0 : trail.n;
      const r = await fetch(`/track?since=${since}&id=${encodeURIComponent(trail.id || "")}&_=${Date.now()}`, { cache: "no-store" });
      const j = await r.json();
      if (j.from === 0 || j.id !== trail.id) { trail.segs = []; trail.segIds = []; }
      for (const [la, lo, sg] of j.pts) {
        if (trail.segIds[trail.segIds.length - 1] !== sg) { trail.segs.push([]); trail.segIds.push(sg); }
        trail.segs[trail.segs.length - 1].push([lo, la]);
      }
      trail.id = j.id; trail.n = j.n; trail.dirty = true;
    } catch (e) {
    } finally { trail.loading = false; }
  }

  // reset needs two taps (so a stray tap never wipes a long drive)
  let resetArmed = 0;
  async function resetTrip(btn) {
    const now = Date.now();
    if (now - resetArmed > 3000) {
      resetArmed = now;
      for (const b of [$("btnReset"), $("btnReset2")]) b.classList.add("armed");
      if (btn && btn.id === "btnReset2") btn.textContent = "Tap again to reset";
      toast("Tap again to reset trip & route");
      setTimeout(disarmReset, 3000);
      return;
    }
    disarmReset();
    try {
      const r = await fetch("/reset", { method: "POST" });
      applySummary(await r.json());
      trail.segs = []; trail.segIds = []; trail.n = 0; trail.dirty = true;
      await loadTrack(true);
      toast("Trip & route reset (total km kept)");
    } catch (e) { toast("Reset failed: PC script not reachable"); }
  }
  function disarmReset() {
    resetArmed = 0;
    for (const b of [$("btnReset"), $("btnReset2")]) b.classList.remove("armed");
    $("btnReset2").textContent = "Reset trip & route";
  }

  function applySummary(j) {
    if (!j) return;
    if (j.trip) { trip.dist = +j.trip.dist_m || 0; trip.moving = +j.trip.moving_s || 0; trip.max = +j.trip.max_ms || 0; }
    if (typeof j.odo_m === "number") trip.odo = j.odo_m;
    if (j.track_id && (j.track_id !== trail.id || j.track_n > trail.n)) loadTrack(j.track_id !== trail.id);
    else if (j.track_id && j.track_n < trail.n) loadTrack(true);
    if (j.fog) onFogInfo(j.fog);
    if (j.update) showUpdate(j.update);
  }

  // ---------------------------------------------------------------- polling the PC script
  async function poll() {
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), 1500);
    try {
      const r = await fetch("/?_=" + Date.now(), { cache: "no-store", signal: ctrl.signal });
      const j = await r.json();
      st.fails = 0;
      applySummary(j);
      if (j && typeof j.lat === "number" && typeof j.lon === "number") onSample(j);
      else if (!st.hasFix) setStatus("waiting");
    } catch (e) {
      if (++st.fails >= 4) setStatus("offline");
    } finally {
      clearTimeout(timer);
      setTimeout(poll, POLL_MS);
    }
  }

  let lastSampleAt = 0;
  function onSample(j) {
    const age = typeof j.age === "number" ? j.age : 0;
    st.stale = age > STALE_S;
    setStatus(st.stale ? "stale" : "live");
    if (j.t === st.lastT) return;
    st.lastT = j.t;

    const now = performance.now();
    const s = {
      lng: j.lon, lat: j.lat, ve: +j.ve || 0, vn: +j.vn || 0,
      speed: +j.speed_ms || 0, bearing: +j.bearing || 0, at: now - age * 1000,
    };
    const p = [s.lng, s.lat];

    // jump instead of glide when: the script saw a teleport (new route segment), or this page was in the
    // background / lost samples for a while (the car is somewhere else by now)
    const teleport = j.event === "jump" || (trail.seg !== null && j.seg !== trail.seg);
    const gap = now - lastSampleAt > SNAP_GAP_MS || document.hidden;
    const snap = teleport || gap || !st.hasFix;
    trail.seg = j.seg; lastSampleAt = now;

    st.sample = s;
    if (st.disp && !snap) {
      const tgt = predict(s, now);
      st.off = [st.disp[0] - tgt[0], st.disp[1] - tgt[1]];
      st.offAt = now;
    } else {
      st.off = [0, 0];
      st.disp = p;
      if (st.hasFix && follow && !document.hidden) map.jumpTo({ center: p });
    }
    if (s.speed > 1.0) st.bearing = s.bearing;
    if (snap) st.dispBearing = st.bearing;
    st.speed = s.speed;
    trail.dirty = true;

    if (!st.hasFix) firstFix(p);
    st.hasFix = true;
  }

  function predict(s, now) {
    const dt = clamp((now - s.at) / 1000, 0, 1.0);
    return [
      s.lng + s.ve * dt / (111320 * Math.cos(toRad(s.lat))),
      s.lat + s.vn * dt / 111320,
    ];
  }

  function setStatus(s) {
    if (st.status === s) return;
    st.status = s;
    const el = $("status"); el.dataset.s = s;
    $("statusText").textContent = { connecting: "Connecting", offline: "PC offline", waiting: "Waiting", live: "Live", stale: "No data" }[s];
    if (marker) marker.getElement().classList.toggle("stale", s !== "live");
    const w = $("waiting");
    if (st.hasFix) { w.classList.add("hidden"); return; }
    w.classList.remove("hidden");
    $("waitTitle").textContent = s === "offline" ? "PC script not reachable" : s === "waiting" ? "Waiting for BeamNG.Earth" : "Connecting…";
    $("waitText").textContent = s === "offline"
      ? "Is bne_live_coords.py still running on your PC? This page reconnects automatically."
      : s === "waiting"
        ? "The script is running. Start BeamNG.drive with -bngeport 47614, load BeamNG.Earth and drive."
        : "Looking for the BeamNG.Earth Live GPS script on your PC.";
  }

  // ---------------------------------------------------------------- trail (route from the script)
  function trailGeoJSON() {
    const lines = trail.segs.filter(x => x.length > 1).map(x => x.slice());
    // let the line end at the car: only within the same segment and when close (never across a teleport)
    const last = trail.segs[trail.segs.length - 1];
    if (st.disp && last && last.length && trail.segIds[trail.segIds.length - 1] === trail.seg) {
      const end = last[last.length - 1];
      if (dist(end, st.disp) < JOIN_MAX_M) {
        if (last.length > 1) lines[lines.length - 1].push(st.disp);
        else lines.push([end, st.disp]);
      }
    }
    return { type: "Feature", properties: {}, geometry: { type: "MultiLineString", coordinates: lines } };
  }

  // ---------------------------------------------------------------- map
  if (typeof maplibregl === "undefined") {
    $("waitTitle").textContent = "Map could not load";
    $("waitText").textContent = "This page needs an internet connection for the map. Check your connection and reload.";
    $("waiting").querySelector(".spinner").classList.add("hidden");
    return;
  }
  const styleOf = id => STYLES.find(s => s.id === id) || STYLES[0];
  const map = new maplibregl.Map({
    container: "map",
    style: styleOf(S.style).url,
    center: [10, 30], zoom: 1.4,
    attributionControl: false,
    maxPitch: 75,
    dragRotate: true,
    pitchWithRotate: true,
  });
  map.touchZoomRotate.enableRotation();

  // car marker
  const carEl = document.createElement("div");
  carEl.className = "car stale";
  carEl.innerHTML = '<div class="halo"></div><svg viewBox="0 0 24 24"><path d="M12 2.5 19 20.5 12 16.6 5 20.5Z" fill="#ff7a1a" stroke="#fff" stroke-width="1.6" stroke-linejoin="round"/></svg>';
  const marker = new maplibregl.Marker({ element: carEl, rotationAlignment: "map", pitchAlignment: "map" });
  let markerAdded = false;

  let styleError = false;
  map.on("error", e => {
    if (styleError) return;
    const msg = String((e && e.error && e.error.message) || "");
    if (/style|Failed to fetch|NetworkError|404/i.test(msg)) { styleError = true; toast("Map data could not load. Check your internet connection."); }
  });

  // Where our 3D buildings and route go: ABOVE every ground layer (land, water, roads, flat buildings) and below
  // the labels only. 2D layers drawn after a fill-extrusion are painted over it without depth test, which made
  // roads and the dark ground show on top of the buildings.
  const LYR_TINT_LC = "bne-tint-landcover", LYR_TINT_PARK = "bne-tint-park";
  const OWN = new Set([LYR_BLD, LYR_TRAIL, LYR_TRAIL_GLOW, LYR_TINT_LC, LYR_TINT_PARK]);

  // ---- Dark style: a little colour for water and nature (subtle, it's still a dark map)
  const TINT = {
    water: "#16303f", waterway: "#1d4057",
    landcover: ["match", ["get", "class"],
      "wood", "#16261b", "forest", "#16261b",
      "grass", "#18241c", "farmland", "#1a211b",
      "wetland", "#152628", "sand", "#29261d", "beach", "#29261d",
      "ice", "#26303a", "glacier", "#26303a", "rock", "#1d1f22",
      "#171d19"],
    park: "#17251b",
  };
  function tintDarkStyle(vsrc) {
    const layers = (map.getStyle() && map.getStyle().layers) || [];
    for (const l of layers) {
      const sl = l["source-layer"];
      try {
        if (sl === "water" && l.type === "fill") map.setPaintProperty(l.id, "fill-color", TINT.water);
        else if (sl === "waterway" && l.type === "line") map.setPaintProperty(l.id, "line-color", TINT.waterway);
        else if (sl === "landcover" && l.type === "fill") map.setPaintProperty(l.id, "fill-color", TINT.landcover);
        else if (sl === "park" && l.type === "fill") map.setPaintProperty(l.id, "fill-color", TINT.park);
      } catch (e) {}
    }
    if (!vsrc) return;
    // the dark style may not draw nature at all: add it right above the background, under everything else
    const firstNonBg = layers.find(l => l.type !== "background");
    const before = firstNonBg ? firstNonBg.id : undefined;
    try {
      if (!map.getLayer(LYR_TINT_LC)) map.addLayer({
        id: LYR_TINT_LC, type: "fill", source: vsrc, "source-layer": "landcover",
        filter: ["in", ["get", "class"], ["literal", ["wood", "forest", "grass", "wetland", "sand", "beach", "ice", "glacier"]]],
        paint: { "fill-color": TINT.landcover, "fill-antialias": false },
      }, before);
      if (!map.getLayer(LYR_TINT_PARK)) map.addLayer({
        id: LYR_TINT_PARK, type: "fill", source: vsrc, "source-layer": "park",
        paint: { "fill-color": TINT.park, "fill-opacity": 0.8, "fill-antialias": false },
      }, before);
    } catch (e) {}
  }
  function overlayBeforeId() {
    const layers = ((map.getStyle() && map.getStyle().layers) || []).filter(l => !OWN.has(l.id));
    let last = -1;
    layers.forEach((l, i) => { if (l.type !== "symbol") last = i; });
    const next = layers[last + 1];
    return next ? next.id : undefined;   // undefined = add on top
  }
  function vectorSourceId() {
    const src = (map.getStyle() && map.getStyle().sources) || {};
    if (src.openmaptiles && src.openmaptiles.type === "vector") return "openmaptiles";
    return Object.keys(src).find(k => src[k].type === "vector");
  }
  let nativeBuildingLayers = [];
  function addOverlays() {
    if (S.style === "dark") tintDarkStyle(vectorSourceId());
    const before = overlayBeforeId();
    const dark = styleOf(S.style).dark;

    // 3D buildings: reuse the style's own extrusion layers if it has them, else add ours
    const layers = (map.getStyle() && map.getStyle().layers) || [];
    nativeBuildingLayers = layers.filter(l => l.type === "fill-extrusion").map(l => l.id);
    const vsrc = vectorSourceId();
    if (!nativeBuildingLayers.length && vsrc && !map.getLayer(LYR_BLD)) {
      map.addLayer({
        id: LYR_BLD, type: "fill-extrusion", source: vsrc, "source-layer": "building", minzoom: 14,
        paint: {
          "fill-extrusion-color": dark ? BLD_DARK : BLD_LIGHT,
          "fill-extrusion-opacity": 1,
          "fill-extrusion-height": ["interpolate", ["linear"], ["zoom"], 14, 0, 15.5, ["coalesce", ["get", "render_height"], 8]],
          "fill-extrusion-base": ["interpolate", ["linear"], ["zoom"], 14, 0, 15.5, ["coalesce", ["get", "render_min_height"], 0]],
        },
      }, before);
    }
    // the style's own extrusion layers: move them on top of the ground layers too
    for (const id of nativeBuildingLayers) { try { map.moveLayer(id, before); } catch (e) {} }
    // the style's own extrusion layers: make them solid (translucent walls look inside-out) and neutral in dark styles
    for (const id of nativeBuildingLayers) {
      try {
        map.setPaintProperty(id, "fill-extrusion-opacity", 1);
        if (dark) map.setPaintProperty(id, "fill-extrusion-color", BLD_DARK);
        map.setPaintProperty(id, "fill-extrusion-vertical-gradient", false);
      } catch (e) {}
    }
    // light from above: roofs brighter than walls, so buildings never look hollow
    // anchored to the MAP (not the screen): with a tilted camera a screen-anchored light hits the walls and leaves roofs dark
    try { map.setLight({ anchor: "map", color: "#ffffff", intensity: dark ? 0.3 : 0.35, position: [1.5, 200, 20] }); } catch (e) {}
    applyBuildings();

    // driven route
    if (!map.getSource(SRC_TRAIL)) map.addSource(SRC_TRAIL, { type: "geojson", data: trailGeoJSON() });
    if (!map.getLayer(LYR_TRAIL_GLOW)) map.addLayer({
      id: LYR_TRAIL_GLOW, type: "line", source: SRC_TRAIL,
      layout: { "line-cap": "round", "line-join": "round" },
      paint: { "line-color": "#ff7a1a", "line-opacity": 0.25, "line-blur": 2,
        "line-width": ["interpolate", ["linear"], ["zoom"], 5, 4, 12, 9, 17, 20] },
    }, before);
    if (!map.getLayer(LYR_TRAIL)) map.addLayer({
      id: LYR_TRAIL, type: "line", source: SRC_TRAIL,
      layout: { "line-cap": "round", "line-join": "round" },
      paint: { "line-color": "#ff7a1a", "line-opacity": 0.95,
        "line-width": ["interpolate", ["linear"], ["zoom"], 5, 1.5, 12, 3.5, 17, 7] },
    }, before);
    applyTrail();
  }
  function applyBuildings() {
    const vis = S.buildings ? "visible" : "none";
    for (const id of [LYR_BLD, ...nativeBuildingLayers]) if (map.getLayer(id)) map.setLayoutProperty(id, "visibility", vis);
  }
  function applyTrail() {
    const vis = S.trail ? "visible" : "none";
    for (const id of [LYR_TRAIL, LYR_TRAIL_GLOW]) if (map.getLayer(id)) map.setLayoutProperty(id, "visibility", vis);
  }
  map.on("style.load", () => { styleError = false; addOverlays(); });
  // safety net: if our layers ever go missing after a style change, add them again
  map.on("styledata", () => {
    if (map.isStyleLoaded() && (!map.getSource(SRC_TRAIL) || !map.getLayer(LYR_TRAIL))) addOverlays();
  });

  function setStyle(id) {
    if (S.style === id) return;
    S.style = id; save(); fog.dirty = true;
    map.setStyle(styleOf(id).url, { diff: false });   // full reload -> "style.load" re-adds route + buildings
    renderStyleList();
  }

  // ---------------------------------------------------------------- fog of the world
  // The script keeps the explored cells (zoom-20 map tiles, ~25 m). Normal view: a translucent mist with soft edges
  // over everything not explored yet. Erase mode: grid + explored cells are shown, tap a cell to remove it.
  const FOG_N = 1048576, FOG_Z = 20, FOG_MIN_PX = 10, FOG_MAX_VERTS = 30000, BRUSH = [0, 3, 8];
  const fog = { epoch: 0, keys: [], set: new Set(), levels: {}, loading: false, dirty: true, edit: false, brush: 0, undo: [], sig: "", info: null };
  const fogCv = document.createElement("canvas");
  fogCv.id = "fog";
  map.getCanvas().after(fogCv);                      // above the map, below the car marker
  const fogMask = document.createElement("canvas"), fogBlur = document.createElement("canvas");
  const xOfLng = lng => (lng + 180) / 360 * FOG_N;
  const yOfLat = lat => { const sn = Math.sin(clamp(lat, -85.0511, 85.0511) * Math.PI / 180); return (0.5 - Math.log((1 + sn) / (1 - sn)) / (4 * Math.PI)) * FOG_N; };
  const parentKey = (k, L) => (Math.floor(k / FOG_N) >> L) * FOG_N + ((k % FOG_N) >> L);

  function fogAdd(k) {
    if (fog.set.has(k)) return;
    fog.set.add(k); fog.keys.push(k);
    for (const L in fog.levels) fog.levels[L].add(parentKey(k, +L));
  }
  function fogLevel(L) {                              // zoomed far out: cells are merged into bigger ones
    if (!L) return fog.set;
    let sset = fog.levels[L];
    if (!sset) { sset = fog.levels[L] = new Set(); for (const k of fog.keys) sset.add(parentKey(k, L)); }
    return sset;
  }
  async function loadFog() {
    if (fog.loading) return;
    fog.loading = true;
    try {
      const r = await fetch(`/fog?epoch=${fog.epoch}&n=${fog.keys.length}&_=${Date.now()}`, { cache: "no-store" });
      const j = await r.json();
      if (j.full) { fog.set = new Set(); fog.keys = []; fog.levels = {}; }
      for (const k of j.cells) fogAdd(k);
      fog.epoch = j.epoch; fog.dirty = true; map.triggerRepaint();
    } catch (e) {
    } finally { fog.loading = false; }
  }
  function onFogInfo(i) {
    fog.info = i;
    $("fogInfo").textContent = `Explored: ${i.km2.toFixed(2)} km² · ${i.cells.toLocaleString()} cells`;
    if (i.epoch !== fog.epoch || i.cells !== fog.keys.length) loadFog();
  }

  // Does the map's own matrix agree with map.project? Then we project grid corners ourselves (cheap, and we can
  // see "behind the camera"); otherwise fall back to map.project.
  function ownProjectionOk(T) {
    const m = T && T.pixelMatrix, ws = T && T.worldSize;
    if (!m || !ws) return false;
    const c = map.getCenter(), mc = maplibregl.MercatorCoordinate.fromLngLat(c), p = map.project(c);
    const X = mc.x * ws, Y = mc.y * ws, w = m[3] * X + m[7] * Y + m[15];
    return w > 0 && Math.abs((m[0] * X + m[4] * Y + m[12]) / w - p.x) < 1.5 && Math.abs((m[1] * X + m[5] * Y + m[13]) / w - p.y) < 1.5;
  }

  function drawFog() {
    fog.dirty = false;
    const show = S.fog || fog.edit;
    fogCv.style.display = show ? "block" : "none";
    if (!show) return;
    const W = map.getCanvas().clientWidth, H = map.getCanvas().clientHeight;
    if (!W || !H) return;
    const edit = fog.edit, dark = styleOf(S.style).dark;
    const FS = edit ? Math.min(window.devicePixelRatio || 1, 2) : 0.5;       // the mist is soft: half resolution is enough
    const w = Math.ceil(W * FS), h = Math.ceil(H * FS);
    if (fogCv.width !== w || fogCv.height !== h) { fogCv.width = w; fogCv.height = h; }
    const ctx = fogCv.getContext("2d");
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalCompositeOperation = "source-over";
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = dark ? (edit ? "rgba(8,11,17,.45)" : "rgba(8,11,17,.76)") : (edit ? "rgba(210,216,226,.5)" : "rgba(210,216,226,.82)");
    ctx.fillRect(0, 0, w, h);

    // which grid cells are on screen (coarser when zoomed out, so there are never more than FOG_MAX_VERTS corners)
    let b;
    try { b = map.getBounds(); } catch (e) { return; }
    if (!(b.getWest() < b.getEast())) return;
    const cellPx = 512 * Math.pow(2, map.getZoom() - FOG_Z);
    let L = 0, s, x0, y0, nx, ny;
    while (L < 18 && cellPx * (1 << L) < FOG_MIN_PX) L++;
    for (;;) {
      s = 1 << L;
      x0 = Math.floor(xOfLng(b.getWest()) / s) - 1;
      y0 = Math.floor(yOfLat(b.getNorth()) / s) - 1;
      nx = Math.floor(xOfLng(b.getEast()) / s) + 2 - x0;
      ny = Math.floor(yOfLat(b.getSouth()) / s) + 2 - y0;
      if (nx * ny <= FOG_MAX_VERTS || L >= 18) break;
      L++;
    }
    const T = map.transform, own = ownProjectionOk(T);
    const m = own ? T.pixelMatrix : null, ws = own ? T.worldSize : 0;
    // a grid point (in cells of this level) -> screen px, or null (behind the camera)
    const pt = (cx, cy) => {
      const u = cx * s / FOG_N, v = cy * s / FOG_N;
      if (own) {
        const X = u * ws, Y = v * ws, wv = m[3] * X + m[7] * Y + m[15];
        return wv > 1 ? [(m[0] * X + m[4] * Y + m[12]) / wv, (m[1] * X + m[5] * Y + m[13]) / wv] : null;
      }
      const p = map.project(new maplibregl.MercatorCoordinate(u, v, 0).toLngLat());
      return Math.abs(p.x) < 8 * W && Math.abs(p.y) < 8 * H ? [p.x, p.y] : null;
    };
    // explored cells; in normal view each is drawn a little bigger, but only when zoomed out so far that a cell is
    // ~25 px or less (close up the opening follows the cells exactly; measured on the ground, so it is right when tilted too)
    const set = fogLevel(L), path = new Path2D(), rows = FOG_N >> L, pad = edit ? 0 : clamp(10 / (cellPx * s) - 0.35, 0, 1);
    let any = false;
    for (let j = 0; j < ny; j++) {
      const yy = y0 + j;
      if (yy < 0 || yy >= rows) continue;
      for (let i = 0; i < nx; i++) {
        if (!set.has((x0 + i) * FOG_N + yy)) continue;
        const cx = x0 + i, A = pt(cx - pad, yy - pad), B = pt(cx + 1 + pad, yy - pad), C = pt(cx + 1 + pad, yy + 1 + pad), D = pt(cx - pad, yy + 1 + pad);
        if (!(A && B && C && D)) continue;
        path.moveTo(A[0], A[1]); path.lineTo(B[0], B[1]); path.lineTo(C[0], C[1]); path.lineTo(D[0], D[1]); path.closePath();
        any = true;
      }
    }
    if (any) {
      // soft edges: draw the explored cells into a small mask, blur it there (3x3 box, twice) so neighbouring cells melt
      // into one smooth opening, scale it up smoothly and cut it out of the mist
      const soft = edit ? 1 : Math.round(clamp(cellPx * s * 0.08, 3, 10));
      const mw = Math.ceil(W / soft), mh = Math.ceil(H / soft);
      for (const c of [fogMask, fogBlur]) if (c.width !== mw || c.height !== mh) { c.width = mw; c.height = mh; }
      const mx = fogMask.getContext("2d");
      mx.setTransform(1, 0, 0, 1, 0, 0);
      mx.clearRect(0, 0, mw, mh);
      mx.setTransform(1 / soft, 0, 0, 1 / soft, 0, 0);
      mx.fillStyle = "#fff";
      mx.fill(path);
      if (!edit) {
        let from = fogMask, to = fogBlur;
        for (let pass = 0; pass < 2; pass++) {
          const bx = to.getContext("2d");
          bx.setTransform(1, 0, 0, 1, 0, 0);
          bx.globalCompositeOperation = "source-over"; bx.globalAlpha = 1; bx.clearRect(0, 0, mw, mh);
          bx.globalCompositeOperation = "lighter"; bx.globalAlpha = 1 / 9;
          for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) bx.drawImage(from, dx, dy);
          bx.globalCompositeOperation = "source-over"; bx.globalAlpha = 1;
          [from, to] = [to, from];
        }
      }
      ctx.globalCompositeOperation = "destination-out";
      ctx.imageSmoothingEnabled = true;
      ctx.imageSmoothingQuality = "high";
      ctx.drawImage(fogMask, 0, 0, mw, mh, 0, 0, mw * soft * FS, mh * soft * FS);
      ctx.globalCompositeOperation = "source-over";
    }
    if (edit) {                                       // the grid is only shown in erase mode
      ctx.setTransform(FS, 0, 0, FS, 0, 0);
      if (any) { ctx.fillStyle = "rgba(255,122,26,.16)"; ctx.fill(path); }
      if (L === 0 && cellPx >= 24) {
        const g = new Path2D();
        for (let j = 0; j <= ny; j++) { const A = pt(x0, y0 + j), B = pt(x0 + nx, y0 + j); if (A && B) { g.moveTo(A[0], A[1]); g.lineTo(B[0], B[1]); } }
        for (let i = 0; i <= nx; i++) { const A = pt(x0 + i, y0), B = pt(x0 + i, y0 + ny); if (A && B) { g.moveTo(A[0], A[1]); g.lineTo(B[0], B[1]); } }
        ctx.lineWidth = 1; ctx.strokeStyle = "rgba(255,255,255,.22)"; ctx.stroke(g);
        if (any) { ctx.lineWidth = 1.5; ctx.strokeStyle = "rgba(255,122,26,.9)"; ctx.stroke(path); }
      }
    }
  }
  function fogSig() {
    const c = map.getCenter(), p = map.getPadding();
    return [c.lng, c.lat, map.getZoom(), map.getBearing(), map.getPitch(), p.top, p.bottom, p.left, p.right,
      map.getCanvas().clientWidth, map.getCanvas().clientHeight].join(",");
  }
  map.on("render", () => {
    const sg = fogSig();
    if (fog.dirty || sg !== fog.sig) { fog.sig = sg; drawFog(); }
  });

  // erase mode: tap cells to remove them (a wrongly explored stretch), undo brings them back
  function renderBrush() { for (const b of $("brushSeg").children) b.classList.toggle("on", +b.dataset.b === fog.brush); }
  function setFogEdit(on) {
    if (OBS) return;
    fog.edit = on; fog.undo = [];
    document.body.classList.toggle("fogedit", on);
    if (on) { closeSheet(); goFree(); renderBrush(); toast(map.getZoom() < 15.5 ? "Zoom in to erase cells" : "Tap cells to erase them"); }
    else if (st.hasFix) recenter();
    fog.dirty = true; map.triggerRepaint();
  }
  async function fogEdit(remove, add) {
    for (const k of remove) fog.set.delete(k);
    for (const k of add) fog.set.add(k);
    fog.keys = Array.from(fog.set); fog.levels = {};   // same order as the script: removed ones gone, restored ones at the end
    fog.dirty = true; map.triggerRepaint();
    try {
      const r = await fetch("/fog/edit", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ remove, add }) });
      fog.epoch = (await r.json()).epoch;
    } catch (e) { toast("Could not save: PC script not reachable"); fog.epoch = 0; loadFog(); }
  }
  map.on("click", e => {
    if (!fog.edit) return;
    if (map.getZoom() < 15.5) { toast("Zoom in to erase cells"); return; }
    const cx = Math.floor(xOfLng(e.lngLat.lng)), cy = Math.floor(yOfLat(e.lngLat.lat)), r = BRUSH[fog.brush], hit = [];
    for (let i = cx - r; i <= cx + r; i++) for (let j = cy - r; j <= cy + r; j++) if (fog.set.has(i * FOG_N + j)) hit.push(i * FOG_N + j);
    if (!hit.length) { toast("Nothing explored here"); return; }
    fog.undo.push({ add: hit }); if (fog.undo.length > 30) fog.undo.shift();
    fogEdit(hit, []);
  });
  let fogResetArmed = 0;
  async function resetFog(btn) {
    const now = Date.now();
    if (now - fogResetArmed > 3000) {
      fogResetArmed = now; btn.classList.add("armed"); btn.textContent = "Tap again to reset";
      toast("Tap again to erase the whole explored map");
      setTimeout(() => { fogResetArmed = 0; btn.classList.remove("armed"); btn.textContent = "Reset fog"; }, 3000);
      return;
    }
    fogResetArmed = 0; btn.classList.remove("armed"); btn.textContent = "Reset fog";
    try {
      const r = await fetch("/fog/reset", { method: "POST" });
      fog.epoch = (await r.json()).epoch;
      fog.set = new Set(); fog.keys = []; fog.levels = {}; fog.undo = []; fog.dirty = true; map.triggerRepaint();
      toast("Fog reset");
    } catch (e) { toast("Reset failed: PC script not reachable"); }
  }

  // ---------------------------------------------------------------- update notice (the script checks GitHub once on start)
  let updShown = false;
  function showUpdate(u) {
    if (updShown || OBS || !u || !u.version) return;
    let gone = "";
    try { gone = localStorage.getItem("bne-upd") || ""; } catch (e) {}
    if (gone === u.version) return;
    updShown = true;
    const a = $("updLink");
    a.textContent = `Update v${u.version} available`;
    if (/^https:\/\/github\.com\//.test(u.url || "")) a.href = u.url;
    $("updPill").classList.remove("hidden");
    document.body.classList.add("upd");
    $("updX").onclick = () => {
      try { localStorage.setItem("bne-upd", u.version); } catch (e) {}
      $("updPill").classList.add("hidden"); document.body.classList.remove("upd");
    };
  }

  // ---------------------------------------------------------------- camera
  let follow = true;                 // false = free mode (user moved the map)
  let interacting = false;
  let zoomOffset = 0;                // user's pinch/scroll correction on top of auto zoom
  let manualZoom = 16;
  let cam = { zoom: 1.4, pitch: 0, bearing: 0 };
  let easingUntil = 0;

  const camPitch = () => S.cam === "chase" ? 62 : (S.buildings ? 50 : 0);
  function zoomForSpeed(kmh) {
    // 0 km/h -> 17, 50 -> 16.3, 100 -> 15.6, 200 -> 14.6, 300+ -> 14
    const k = Math.min(kmh, 400);
    const z = 17 - 0.0145 * k + 0.0000135 * k * k;
    return clamp(z, 13.6, 17.2) + (S.cam === "chase" ? 0.3 : 0);
  }
  let hudCover = 0;                  // px of the screen bottom covered by the HUD (updated in renderHud)
  function hudPadding() {
    const h = window.innerHeight;
    const bottomCover = OBS ? 0 : hudCover;
    if (S.cam === "chase") return { top: Math.round(h * 0.38), bottom: Math.round(bottomCover * 0.5), left: 0, right: 0 };
    if (S.cam === "heading") return { top: Math.round(h * 0.22), bottom: Math.round(bottomCover * 0.6), left: 0, right: 0 };
    return { top: 0, bottom: Math.round(bottomCover * 0.6), left: 0, right: 0 };
  }

  function firstFix(p) {
    $("waiting").classList.add("hidden");
    marker.setLngLat(p).addTo(map); markerAdded = true;
    cam.zoom = S.autoZoom ? zoomForSpeed(0) : manualZoom;
    cam.pitch = camPitch();
    map.jumpTo({ center: p, zoom: cam.zoom, pitch: cam.pitch, bearing: S.cam === "north" ? 0 : st.bearing, padding: hudPadding() });
  }

  function setCam(c, silent) {
    S.cam = c; save(); follow = true; zoomOffset = 0;
    $("recenter").classList.add("hidden");
    renderCamButton(); renderCamSeg();
    if (st.disp) {
      cam.pitch = camPitch();
      easingUntil = performance.now() + 650;
      map.easeTo({ center: st.disp, pitch: cam.pitch, bearing: c === "north" ? 0 : st.dispBearing,
        zoom: S.autoZoom ? zoomForSpeed(spdKmh()) : manualZoom, padding: hudPadding(), duration: 600 });
    }
    if (!silent) toast({ north: "Camera: North up", heading: "Camera: Heading up", chase: "Camera: Chase 3D" }[c]);
  }
  function recenter() { setCam(S.cam, true); toast("Following the car"); }
  const spdKmh = () => st.speed * 3.6;

  // user interaction: drag/rotate/pitch -> free mode; zoom -> remember offset
  const userEvt = e => !!(e && e.originalEvent);
  map.on("dragstart", e => { if (userEvt(e)) goFree(); });
  map.on("rotatestart", e => { if (userEvt(e)) goFree(); });
  map.on("pitchstart", e => { if (userEvt(e)) goFree(); });
  map.on("zoomstart", e => { if (userEvt(e)) interacting = true; });
  map.on("zoomend", e => {
    if (!userEvt(e)) return;
    interacting = false;
    if (S.autoZoom) zoomOffset = clamp(map.getZoom() - zoomForSpeed(spdKmh()), -6, 4);
    else manualZoom = map.getZoom();
    cam.zoom = map.getZoom();
  });
  const pointers = new Set();
  let wheelUntil = 0;
  const box = map.getCanvasContainer();
  box.addEventListener("pointerdown", e => pointers.add(e.pointerId), { passive: true });
  for (const ev of ["pointerup", "pointercancel"]) window.addEventListener(ev, e => pointers.delete(e.pointerId), { passive: true });
  window.addEventListener("blur", () => pointers.clear());
  box.addEventListener("wheel", () => { wheelUntil = performance.now() + 400; }, { passive: true });
  map.on("movestart", e => {                     // pan with mouse/finger -> free mode
    if (userEvt(e) && pointers.size && e.originalEvent.type !== "wheel") goFree();
  });

  function goFree() {
    if (!st.hasFix) return;
    follow = false; interacting = false;
    $("recenter").classList.remove("hidden");
    renderCamButton();
  }

  // ---------------------------------------------------------------- render loop
  let lastFrame = performance.now(), lastTrailUpdate = 0, lastHud = 0;
  function frame(now) {
    requestAnimationFrame(frame);
    const dt = Math.min(0.1, (now - lastFrame) / 1000); lastFrame = now;
    if (!st.sample || document.hidden) return;

    // position: prediction + fading correction
    const base = st.stale ? [st.sample.lng, st.sample.lat] : predict(st.sample, now);
    const fade = Math.exp(-(now - st.offAt) / 280);
    st.disp = [base[0] + st.off[0] * fade, base[1] + st.off[1] * fade];

    // bearing: shortest-way smoothing
    st.dispBearing = (st.dispBearing + angDiff(st.dispBearing, st.bearing) * (1 - Math.exp(-dt / 0.22)) + 360) % 360;

    if (markerAdded) { marker.setLngLat(st.disp); marker.setRotation(st.dispBearing); }

    if (follow && !interacting && !pointers.size && now > wheelUntil && now > easingUntil) {
      const targetZoom = S.autoZoom ? zoomForSpeed(st.stale ? 0 : spdKmh()) + zoomOffset : manualZoom;
      cam.zoom = lerp(cam.zoom, targetZoom, 1 - Math.exp(-dt / 0.9));
      cam.pitch = lerp(cam.pitch, camPitch(), 1 - Math.exp(-dt / 0.3));
      cam.bearing = S.cam === "north" ? 0 : st.dispBearing;
      map.jumpTo({ center: st.disp, zoom: cam.zoom, pitch: cam.pitch, bearing: cam.bearing, padding: hudPadding() });
    }

    if (trail.dirty || now - lastTrailUpdate > 400) {
      if (now - lastTrailUpdate > 250) {
        const src = map.getSource(SRC_TRAIL);
        if (src && S.trail) src.setData(trailGeoJSON());
        trail.dirty = false; lastTrailUpdate = now;
      }
    }
    if (now - lastHud > 100) { renderHud(); lastHud = now; }
  }
  requestAnimationFrame(frame);

  // ---------------------------------------------------------------- HUD
  let shownSpeed = 0;
  function renderHud() {
    const target = st.stale ? 0 : spdOut(st.speed);
    shownSpeed = Math.abs(target - shownSpeed) < 0.6 ? target : lerp(shownSpeed, target, 0.45);
    $("spd").textContent = Math.round(shownSpeed);
    $("unit").textContent = spdUnit();
    $("needle").style.transform = `rotate(${st.dispBearing}deg)`;
    $("hdg").textContent = `${compass(st.dispBearing)} ${String(Math.round(st.dispBearing) % 360).padStart(3, "0")}°`;
    if (st.disp) $("coords").textContent = `${st.disp[1].toFixed(5)}, ${st.disp[0].toFixed(5)}`;
    renderStats();
    const r = $("hud").getBoundingClientRect();
    hudCover = Math.max(0, window.innerHeight - r.top);
    document.documentElement.style.setProperty("--hud-h", r.height + "px");
  }
  function renderStats() {
    $("stDist").textContent = distOut(trip.dist);
    $("stTime").textContent = fmtTime(trip.moving);
    const avg = trip.moving > 1 ? Math.min(trip.dist / trip.moving, trip.max || Infinity) : 0;
    $("stAvg").textContent = `${Math.round(spdOut(avg))} ${spdUnit()}`;
    $("stMax").textContent = `${Math.round(spdOut(trip.max))} ${spdUnit()}`;
    $("stOdo").textContent = odoOut(trip.odo);
    $("odoBig").textContent = odoOut(trip.odo, 1);
    $("tripInfo").textContent = `Current trip: ${distOut(trip.dist)} · drive time ${fmtTime(trip.moving)} · top ${Math.round(spdOut(trip.max))} ${spdUnit()}`;
  }
  function odoOut(m, dec) {
    const v = imperial() ? m / 1609.344 : m / 1000;
    const d = dec !== undefined ? dec : (v < 100 ? 1 : 0);
    return v.toLocaleString("en-US", { minimumFractionDigits: d, maximumFractionDigits: d }) + (imperial() ? " mi" : " km");
  }


  // ---------------------------------------------------------------- controls
  const CAM_ICONS = {
    north: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"><circle cx="12" cy="12" r="9"/><path d="M12 5.5 15 13h-6l3-7.5Z" fill="currentColor"/><path d="M12 18.5 9 13h6l-3 5.5Z" opacity=".4"/></svg>',
    heading: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"><path d="M12 3 19 20 12 16 5 20Z" fill="currentColor" fill-opacity=".2"/></svg>',
    chase: '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"><path d="M3 20 8 9h8l5 11Z" fill="currentColor" fill-opacity=".15"/><path d="M12 9.5 14.5 16 12 14.6 9.5 16Z" fill="currentColor"/><path d="M8 5h8" opacity=".6"/></svg>',
  };
  function renderCamButton() { $("btnCam").innerHTML = CAM_ICONS[S.cam]; $("btnCam").classList.toggle("on", follow && S.cam !== "north"); }
  function renderCamSeg() { for (const b of $("camSeg").children) b.classList.toggle("on", b.dataset.cam === S.cam); }
  function render3d() { $("btn3d").classList.toggle("on", S.buildings); }
  function renderSwitches() {
    for (const sw of document.querySelectorAll(".switch")) sw.setAttribute("aria-checked", String(!!S[sw.dataset.opt]));
    for (const b of $("unitSeg").children) b.classList.toggle("on", b.dataset.unit === S.units);
    $("stats").classList.toggle("hidden", !S.stats && !OBS);
  }
  function renderStyleList() {
    const box = $("styleList"); box.innerHTML = "";
    for (const s of STYLES) {
      const b = document.createElement("button");
      b.className = "sty" + (s.id === S.style ? " on" : "");
      b.innerHTML = `<i style="background:${s.sw}"></i><span>${s.name}</span>`;
      b.onclick = () => setStyle(s.id);
      box.appendChild(b);
    }
  }

  $("btnCam").onclick = () => setCam(follow ? CAMS[(CAMS.indexOf(S.cam) + 1) % CAMS.length] : S.cam);
  $("recenter").onclick = recenter;
  function toggle3d() {
    S.buildings = !S.buildings; save(); applyBuildings(); render3d(); renderSwitches();
    if (!follow && st.hasFix) map.easeTo({ pitch: camPitch(), duration: 500 });   // free mode: tilt now
    toast(S.buildings ? "3D on" : "3D off");
  }
  $("btn3d").onclick = toggle3d;
  $("btnStyle").onclick = () => {
    if (window.innerWidth >= 760) { openSheet(); return; }
    const i = STYLES.findIndex(s => s.id === S.style);
    const next = STYLES[(i + 1) % STYLES.length]; setStyle(next.id); toast("Map: " + next.name);
  };
  $("btnSettings").onclick = () => $("sheet").classList.contains("hidden") ? openSheet() : closeSheet();
  $("btnClose").onclick = closeSheet;
  $("scrim").onclick = closeSheet;
  function openSheet() { renderStyleList(); renderSwitches(); renderCamSeg(); $("sheet").classList.remove("hidden"); $("scrim").classList.remove("hidden"); }
  function closeSheet() { $("sheet").classList.add("hidden"); $("scrim").classList.add("hidden"); }

  $("camSeg").onclick = e => { const c = e.target.dataset && e.target.dataset.cam; if (c) setCam(c, true); };
  $("unitSeg").onclick = e => { const u = e.target.dataset && e.target.dataset.unit; if (u) { S.units = u; save(); renderSwitches(); renderHud(); } };
  for (const sw of document.querySelectorAll(".switch")) {
    sw.onclick = () => {
      const k = sw.dataset.opt; S[k] = !S[k]; save(); renderSwitches();
      if (k === "buildings") { S.buildings = !S.buildings; toggle3d(); return; }
      if (k === "trail") { applyTrail(); trail.dirty = true; }
      if (k === "fog") { fog.dirty = true; map.triggerRepaint(); }
      if (k === "autoZoom") { zoomOffset = 0; manualZoom = map.getZoom(); }
      if (k === "awake") { if (S.awake) { keepAwake(); toast("Screen stays on"); } else { releaseAwake(); toast("Screen can turn off"); } updateAwakeHint(); }
    };
  }
  $("spdBox").onclick = () => { S.units = imperial() ? "metric" : "imperial"; save(); renderSwitches(); renderHud(); toast(spdUnit()); };
  $("btnReset").onclick = e => resetTrip(e.currentTarget);
  $("btnReset2").onclick = e => resetTrip(e.currentTarget);
  $("btnFogEdit").onclick = () => setFogEdit(true);
  $("btnFogReset").onclick = e => resetFog(e.currentTarget);
  $("fogDone").onclick = () => setFogEdit(false);
  $("fogUndo").onclick = () => { const u = fog.undo.pop(); if (u) fogEdit([], u.add); else toast("Nothing to undo"); };
  $("brushSeg").onclick = e => { const b = e.target.dataset && e.target.dataset.b; if (b !== undefined) { fog.brush = +b; renderBrush(); } };

  // copy coordinates (clipboard API needs HTTPS, so fall back for the Wi-Fi address)
  $("coords").onclick = () => {
    if (!st.disp) return;
    const txt = `${st.disp[1].toFixed(6)}, ${st.disp[0].toFixed(6)}`;
    const done = () => toast("Copied " + txt);
    if (navigator.clipboard && window.isSecureContext) { navigator.clipboard.writeText(txt).then(done, () => legacyCopy(txt) && done()); }
    else if (legacyCopy(txt)) done();
  };
  function legacyCopy(txt) {
    const ta = document.createElement("textarea"); ta.value = txt; ta.setAttribute("readonly", "");
    ta.style.position = "fixed"; ta.style.opacity = "0"; document.body.appendChild(ta); ta.select();
    let ok = false; try { ok = document.execCommand("copy"); } catch (e) {}
    ta.remove(); return ok;
  }

  // fullscreen (not available for web pages on iPhone: hide the button there)
  const root = document.documentElement;
  const canFull = !!(root.requestFullscreen || root.webkitRequestFullscreen);
  if (!canFull) $("btnFull").classList.add("hidden");
  function toggleFull() {
    const fsEl = document.fullscreenElement || document.webkitFullscreenElement;
    if (fsEl) (document.exitFullscreen || document.webkitExitFullscreen).call(document);
    else (root.requestFullscreen || root.webkitRequestFullscreen).call(root);
  }
  $("btnFull").onclick = toggleFull;

  // keyboard
  window.addEventListener("keydown", e => {
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    const k = e.key.toLowerCase();
    if (k === "c") $("btnCam").click();
    else if (k === "b") $("btn3d").click();
    else if (k === "s") { const i = STYLES.findIndex(s => s.id === S.style); setStyle(STYLES[(i + 1) % STYLES.length].id); }
    else if (k === "r") recenter();
    else if (k === "f" && canFull) toggleFull();
    else if (k === "u") $("spdBox").click();
    else if (k === "escape") closeSheet();
  });

  // ---------------------------------------------------------------- keep the screen on
  // The Wake Lock API only works on HTTPS (or on the PC itself). Over the Wi-Fi address the page falls back to a
  // looping video. Browsers only keep the screen on for a video that is (1) playing, (2) NOT muted (it has a silent
  // audio track, so nothing is heard) and (3) visible and large in the viewport. So the video fills the whole screen
  // BEHIND the map (the map covers it). Playing unmuted needs a tap, so it starts with the first touch/click.
  const NOSLEEP_MP4 = "data:video/mp4;base64,AAAAIGZ0eXBpc29tAAACAGlzb21pc28yYXZjMW1wNDEAAAc6bW9vdgAAAGxtdmhkAAAAAAAAAAAAAAAAAAAD6AAAD6AAAQAAAQAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAwAAAod0cmFrAAAAXHRraGQAAAADAAAAAAAAAAAAAAABAAAAAAAAD6AAAAAAAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAABAAAAAAAAAAAAAAAAAABAAAAAABAAAAAQAAAAAAAkZWR0cwAAABxlbHN0AAAAAAAAAAEAAA+gAAAAAAABAAAAAAH/bWRpYQAAACBtZGhkAAAAAAAAAAAAAAAAAABAAAABAABVxAAAAAAALWhkbHIAAAAAAAAAAHZpZGUAAAAAAAAAAAAAAABWaWRlb0hhbmRsZXIAAAABqm1pbmYAAAAUdm1oZAAAAAEAAAAAAAAAAAAAACRkaW5mAAAAHGRyZWYAAAAAAAAAAQAAAAx1cmwgAAAAAQAAAWpzdGJsAAAAtnN0c2QAAAAAAAAAAQAAAKZhdmMxAAAAAAAAAAEAAAAAAAAAAAAAAAAAAAAAABAAEABIAAAASAAAAAAAAAABFUxhdmM2MC4zMS4xMDIgbGlieDI2NAAAAAAAAAAAAAAAGP//AAAALGF2Y0MBQsAe/+EAFWdCwB7aewEQAAADABAAAAMAQPFi6gEABGjOD8gAAAAQcGFzcAAAAAEAAAABAAAAFGJ0cnQAAAAAAAAFiAAABYgAAAAYc3R0cwAAAAAAAAABAAAACAAAIAAAAAAUc3RzcwAAAAAAAAABAAAAAQAAABxzdHNjAAAAAAAAAAEAAAABAAAAAQAAAAEAAAA0c3RzegAAAAAAAAAAAAAACAAAAoUAAAAJAAAACQAAAAkAAAAJAAAACQAAAAkAAAAJAAAAMHN0Y28AAAAAAAAACAAAB38AAAowAAAKZQAACpoAAArPAAALAAAACzUAAAtqAAAD3XRyYWsAAABcdGtoZAAAAAMAAAAAAAAAAAAAAAIAAAAAAAAPoAAAAAAAAAAAAAAAAQEAAAAAAQAAAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAEAAAAAAAAAAAAAAAAAAACRlZHRzAAAAHGVsc3QAAAAAAAAAAQAAD6AAAAQAAAEAAAAAA1VtZGlhAAAAIG1kaGQAAAAAAAAAAAAAAAAAAFYiAAFciFXEAAAAAAAtaGRscgAAAAAAAAAAc291bgAAAAAAAAAAAAAAAFNvdW5kSGFuZGxlcgAAAAMAbWluZgAAABBzbWhkAAAAAAAAAAAAAAAkZGluZgAAABxkcmVmAAAAAAAAAAEAAAAMdXJsIAAAAAEAAALEc3RibAAAAH5zdHNkAAAAAAAAAAEAAABubXA0YQAAAAAAAAABAAAAAAAAAAAAAQAQAAAAAFYiAAAAAAA2ZXNkcwAAAAADgICAJQACAASAgIAXQBUAAAAAAD6AAAAC2QWAgIAFE4hW5QAGgICAAQIAAAAUYnRydAAAAAAAAD6AAAAC2QAAACBzdHRzAAAAAAAAAAIAAABXAAAEAAAAAAEAAACIAAAAQHN0c2MAAAAAAAAABAAAAAEAAAABAAAAAQAAAAIAAAALAAAAAQAAAAYAAAAKAAAAAQAAAAcAAAALAAAAAQAAAXRzdHN6AAAAAAAAAAAAAABYAAAAFQAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAAAQAAAAEAAAABAAAADRzdGNvAAAAAAAAAAkAAAdqAAAKBAAACjkAAApuAAAKowAACtgAAAsJAAALPgAAC3MAAAAac2dwZAEAAAByb2xsAAAAAgAAAAH//wAAABxzYmdwAAAAAHJvbGwAAAABAAAAWAAAAAEAAABidWR0YQAAAFptZXRhAAAAAAAAACFoZGxyAAAAAAAAAABtZGlyYXBwbAAAAAAAAAAAAAAAAC1pbHN0AAAAJal0b28AAAAdZGF0YQAAAAEAAAAATGF2ZjYwLjE2LjEwMAAAAAhmcmVlAAAEPW1kYXTeAgBMYXZjNjAuMzEuMTAyAAIwQA4AAAJvBgX//2vcRem95tlIt5Ys2CDZI+7veDI2NCAtIGNvcmUgMTY0IHIzMTA4IDMxZTE5ZjkgLSBILjI2NC9NUEVHLTQgQVZDIGNvZGVjIC0gQ29weWxlZnQgMjAwMy0yMDIzIC0gaHR0cDovL3d3dy52aWRlb2xhbi5vcmcveDI2NC5odG1sIC0gb3B0aW9uczogY2FiYWM9MCByZWY9MSBkZWJsb2NrPTE6MDowIGFuYWx5c2U9MHgxOjB4MTExIG1lPWhleCBzdWJtZT0yIHBzeT0xIHBzeV9yZD0xLjAwOjAuMDAgbWl4ZWRfcmVmPTAgbWVfcmFuZ2U9MTYgY2hyb21hX21lPTEgdHJlbGxpcz0wIDh4OGRjdD0wIGNxbT0wIGRlYWR6b25lPTIxLDExIGZhc3RfcHNraXA9MSBjaHJvbWFfcXBfb2Zmc2V0PTAgdGhyZWFkcz0xIGxvb2thaGVhZF90aHJlYWRzPTEgc2xpY2VkX3RocmVhZHM9MCBucj0wIGRlY2ltYXRlPTEgaW50ZXJsYWNlZD0wIGJsdXJheV9jb21wYXQ9MCBjb25zdHJhaW5lZF9pbnRyYT0wIGJmcmFtZXM9MCB3ZWlnaHRwPTAga2V5aW50PTI1MCBrZXlpbnRfbWluPTIgc2NlbmVjdXQ9NDAgaW50cmFfcmVmcmVzaD0wIHJjX2xvb2thaGVhZD0xMCByYz1jcmYgbWJ0cmVlPTEgY3JmPTIzLjAgcWNvbXA9MC42MCBxcG1pbj0wIHFwbWF4PTY5IHFwc3RlcD00IGlwX3JhdGlvPTEuNDAgYXE9MToxLjAwAIAAAAAOZYiEBX///w9FAAFC34ABGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwAAAAVBmiAV9QEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHAAAABUGaQBX1ARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcAAAAFQZpgFfUBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwAAAAVBmoAU9QEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcAAAAFQZqgFPUBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwAAAAVBmsAU9QEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHAAAABUGa4BL1ARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAcBGCAHARggBwEYIAc=";
  const NOSLEEP_WEBM = "data:video/webm;base64,GkXfo59ChoEBQveBAULygQRC84EIQoKEd2VibUKHgQRChYECGFOAZwEAAAAAAAy7EU2bdLpNu4tTq4QVSalmU6yBoU27i1OrhBZUrmtTrIHYTbuMU6uEElTDZ1OsggGETbuMU6uEHFO7a1Osggyl7AEAAAAAAABZAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAVSalmsirXsYMPQkBNgI1MYXZmNjAuMTYuMTAwV0GNTGF2ZjYwLjE2LjEwMESJiECvUAAAAAAAFlSua0CmrgEAAAAAAAA414EBc8WIHpFOScVNCKScgQAitZyDdW5kiIEAhoVWX1ZQOIOBASPjg4QdzWUA4ImwgRC6gRCagQKuAQAAAAAAAFzXgQJzxYiQQfkJS021e5yBACK1nIN1bmSIgQCGhkFfT1BVU1aqg2MuoFa7hATEtACDgQLhkZ+BAbWIQOdwAAAAAABiZIEQY6KTT3B1c0hlYWQBATgBgLsAAAAAABJUw2dA1nNzoGPAgGfImkWjh0VOQ09ERVJEh41MYXZmNjAuMTYuMTAwc3PWY8CLY8WIHpFOScVNCKRnyKFFo4dFTkNPREVSRIeUTGF2YzYwLjMxLjEwMiBsaWJ2cHhnyKFFo4hEVVJBVElPTkSHkzAwOjAwOjA0LjAwMDAwMDAwMABzc9djwItjxYiQQfkJS021e2fIokWjh0VOQ09ERVJEh5VMYXZjNjAuMzEuMTAyIGxpYm9wdXNnyKFFo4hEVVJBVElPTkSHkzAwOjAwOjA0LjAwODAwMDAwMAAfQ7Z1Sj/ngQCji4IAAIAIC+S5oLyEo6OBAACAEAIAnQEqEAAQAABHCIWFiIWEiAICAAwNYAD+/6tQgKOKggAVgAgHxrMOxqOKggApgAgHxrMOxqOKggA9gAgHxrMOxqOKggBRgAgHxrMOxqOKggBlgAgHxrMOxqOKggB5gAgHxrMOxqOKggCNgAgHxrMOxqOKggChgAgHxrMOxqOKggC1gAgHxrMOxqOKggDJgAgHxrMOxqOKggDdgAgHxrMOxqOKggDxgAgHxrMOxqOKggEFgAgHxrMOxqOKggEZgAgHxrMOxqOKggEtgAgHxrMOxqOKggFBgAgHxrMOxqOKggFVgAgHxrMOxqOKggFpgAgHxrMOxqOKggF9gAgHxrMOxqOKggGRgAgHxrMOxqOKggGlgAgHxrMOxqOKggG5gAgHxrMOxqOKggHNgAgHxrMOxqOKggHhgAgHxrMOxqOKggH1gAgHxrMOxqOVgQH0ALEBAAEQEAAYABhYL/QACAAAo4qCAgmACAfGsw7Go4qCAh2ACAfGsw7Go4qCAjGACAfGsw7Go4qCAkWACAfGsw7Go4qCAlmACAfGsw7Go4qCAm2ACAfGsw7Go4qCAoGACAfGsw7Go4qCApWACAfGsw7Go4qCAqmACAfGsw7Go4qCAr2ACAfGsw7Go4qCAtGACAfGsw7Go4qCAuWACAfGsw7Go4qCAvmACAfGsw7Go4qCAw2ACAfGsw7Go4qCAyGACAfGsw7Go4qCAzWACAfGsw7Go4qCA0mACAfGsw7Go4qCA12ACAfGsw7Go4qCA3GACAfGsw7Go4qCA4WACAfGsw7Go4qCA5mACAfGsw7Go4qCA62ACAfGsw7Go4qCA8GACAfGsw7Go4qCA9WACAfGsw7Go4qCA+mACAfGsw7Go5WBA+gAsQEAARAQABgAGFgv9AAIAACjioID/YAIB8azDsajioIEEYAIB8azDsajioIEJYAIB8azDsajioIEOYAIB8azDsajioIETYAIB8azDsajioIEYYAIB8azDsajioIEdYAIB8azDsajioIEiYAIB8azDsajioIEnYAIB8azDsajioIEsYAIB8azDsajioIExYAIB8azDsajioIE2YAIB8azDsajioIE7YAIB8azDsajioIFAYAIB8azDsajioIFFYAIB8azDsajioIFKYAIB8azDsajioIFPYAIB8azDsajioIFUYAIB8azDsajioIFZYAIB8azDsajioIFeYAIB8azDsajioIFjYAIB8azDsajioIFoYAIB8azDsajioIFtYAIB8azDsajioIFyYAIB8azDsajioIF3YAIB8azDsajlYEF3ACxAQABEBAAGAAYWC/0AAgAAKOKggXxgAgHxrMOxqOKggYFgAgHxrMOxqOKggYZgAgHxrMOxqOKggYtgAgHxrMOxqOKggZBgAgHxrMOxqOKggZVgAgHxrMOxqOKggZpgAgHxrMOxqOKggZ9gAgHxrMOxqOKggaRgAgHxrMOxqOKggalgAgHxrMOxqOKgga5gAgHxrMOxqOKggbNgAgHxrMOxqOKggbhgAgHxrMOxqOKggb1gAgHxrMOxqOKggcJgAgHxrMOxqOKggcdgAgHxrMOxqOKggcxgAgHxrMOxqOKggdFgAgHxrMOxqOKggdZgAgHxrMOxqOKggdtgAgHxrMOxqOKggeBgAgHxrMOxqOKggeVgAgHxrMOxqOKggepgAgHxrMOxqOKgge9gAgHxrMOxqOKggfRgAgHxrMOxqOVgQfQALEBAAEQEAAYABhYL/QACAAAo4qCB+WACAfGsw7Go4qCB/mACAfGsw7Go4qCCA2ACAfGsw7Go4qCCCGACAfGsw7Go4qCCDWACAfGsw7Go4qCCEmACAfGsw7Go4qCCF2ACAfGsw7Go4qCCHGACAfGsw7Go4qCCIWACAfGsw7Go4qCCJmACAfGsw7Go4qCCK2ACAfGsw7Go4qCCMGACAfGsw7Go4qCCNWACAfGsw7Go4qCCOmACAfGsw7Go4qCCP2ACAfGsw7Go4qCCRGACAfGsw7Go4qCCSWACAfGsw7Go4qCCTmACAfGsw7Go4qCCU2ACAfGsw7Go4qCCWGACAfGsw7Go4qCCXWACAfGsw7Go4qCCYmACAfGsw7Go4qCCZ2ACAfGsw7Go4qCCbGACAfGsw7Go4qCCcWACAfGsw7Go5WBCcQAsQEAARAQABgAGFgv9AAIAACjioIJ2YAIB8azDsajioIJ7YAIB8azDsajioIKAYAIB8azDsajioIKFYAIB8azDsajioIKKYAIB8azDsajioIKPYAIB8azDsajioIKUYAIB8azDsajioIKZYAIB8azDsajioIKeYAIB8azDsajioIKjYAIB8azDsajioIKoYAIB8azDsajioIKtYAIB8azDsajioIKyYAIB8azDsajioIK3YAIB8azDsajioIK8YAIB8azDsajioILBYAIB8azDsajioILGYAIB8azDsajioILLYAIB8azDsajioILQYAIB8azDsajioILVYAIB8azDsajioILaYAIB8azDsajioILfYAIB8azDsajioILkYAIB8azDsajioILpYAIB8azDsajioILuYAIB8azDsajlYELuACxAQABEBAAGAAYWC/0AAgAAKOKggvNgAgHxrMOxqOKggvhgAgHxrMOxqOKggv1gAgHxrMOxqOKggwJgAgHxrMOxqOKggwdgAgHxrMOxqOKggwxgAgHxrMOxqOKggxFgAgHxrMOxqOKggxZgAgHxrMOxqOKggxtgAgHxrMOxqOKggyBgAgHxrMOxqOKggyVgAgHxrMOxqOKggypgAgHxrMOxqOKggy9gAgHxrMOxqOKggzRgAgHxrMOxqOKggzlgAgHxrMOxqOKggz5gAgHxrMOxqOKgg0NgAgHxrMOxqOKgg0hgAgHxrMOxqOKgg01gAgHxrMOxqOKgg1JgAgHxrMOxqOKgg1dgAgHxrMOxqOKgg1xgAgHxrMOxqOKgg2FgAgHxrMOxqOKgg2ZgAgHxrMOxqOKgg2tgAgHxrMOxqOVgQ2sALEBAAEQEBRgAGFgv9AAIAAAo4qCDcGACAfGsw7Go4qCDdWACAfGsw7Go4qCDemACAfGsw7Go4qCDf2ACAfGsw7Go4qCDhGACAfGsw7Go4qCDiWACAfGsw7Go4qCDjmACAfGsw7Go4qCDk2ACAfGsw7Go4qCDmGACAfGsw7Go4qCDnWACAfGsw7Go4qCDomACAfGsw7Go4qCDp2ACAfGsw7Go4qCDrGACAfGsw7Go4qCDsWACAfGsw7Go4qCDtmACAfGsw7Go4qCDu2ACAfGsw7Go4qCDwGACAfGsw7Go4qCDxWACAfGsw7Go4qCDymACAfGsw7Go4qCDz2ACAfGsw7Go4qCD1GACAfGsw7Go4qCD2WACAfGsw7Go4qCD3mACAfGsw7Go4qCD42ACAfGsw7GoJOhioIPoQAIB8azDsZ1ooQAzf5gHFO7a5G7j7OBALeK94EB8YICYPCBEA==";
  let wakeLock = null, noSleepVideo = null, awakeOn = false;
  function makeNoSleepVideo() {
    const v = document.createElement("video");
    v.setAttribute("playsinline", ""); v.setAttribute("webkit-playsinline", "");
    v.muted = false; v.volume = 1; v.loop = true; v.title = "Keep screen on";
    v.setAttribute("disableremoteplayback", ""); v.disableRemotePlayback = true;
    v.setAttribute("disablepictureinpicture", "");
    v.style.cssText = "position:fixed;inset:0;width:100vw;height:100vh;object-fit:cover;background:#0d1015;pointer-events:none;z-index:0";
    for (const [src, type] of [[NOSLEEP_WEBM, "video/webm"], [NOSLEEP_MP4, "video/mp4"]]) {
      const so = document.createElement("source"); so.src = src; so.type = type; v.appendChild(so);
    }
    // never let it reach the end (some browsers release the lock there)
    v.addEventListener("timeupdate", () => { if (v.currentTime > 2) v.currentTime = 0.5; });
    document.body.insertBefore(v, document.body.firstChild);   // first child = drawn below the map
    return v;
  }
  async function keepAwake() {
    if (!S.awake || document.hidden) return;
    if ("wakeLock" in navigator && window.isSecureContext) {
      try {
        if (!wakeLock || wakeLock.released) wakeLock = await navigator.wakeLock.request("screen");
        awakeOn = true; updateAwakeHint(); return;
      } catch (e) { /* fall through to the video */ }
    }
    if (!noSleepVideo) noSleepVideo = makeNoSleepVideo();
    if (!noSleepVideo.paused) { awakeOn = true; updateAwakeHint(); return; }
    try { await noSleepVideo.play(); awakeOn = true; } catch (e) { awakeOn = false; }   // needs a tap first
    updateAwakeHint();
  }
  // hint on phones/tablets until the screen is actually being kept on
  const touchDevice = matchMedia("(hover: none), (pointer: coarse)").matches;
  let hintReady = false;
  function updateAwakeHint() {
    const show = touchDevice && !OBS && hintReady && S.awake && !awakeOn;
    $("awakeHint").classList.toggle("hidden", !show);
  }
  setTimeout(() => { hintReady = true; updateAwakeHint(); }, 1200);   // give autoplay a moment (e.g. inside the app)
  $("awakeHint").onclick = async () => { await keepAwake(); updateAwakeHint(); if (awakeOn) toast("Screen stays on"); };

  function releaseAwake() {
    awakeOn = false;
    try { if (wakeLock) wakeLock.release(); } catch (e) {}
    wakeLock = null;
    if (noSleepVideo) noSleepVideo.pause();
  }
  // first (and every) tap: start it if it isn't running yet
  for (const ev of ["pointerdown", "touchend", "click", "keydown"]) {
    document.addEventListener(ev, () => { if (S.awake && !awakeOn) keepAwake(); }, { passive: true, capture: true });
  }
  // if the system pauses the video (e.g. other audio starts), start it again on the next tap
  setInterval(() => {
    if (S.awake && noSleepVideo && noSleepVideo.paused && !(wakeLock && !wakeLock.released)) awakeOn = false;
    updateAwakeHint();
  }, 2000);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) { awakeOn = false; return; }
    updateAwakeHint();
    if (S.awake) keepAwake();
  });
  keepAwake();

  window.addEventListener("resize", () => { map.resize(); });

  $("obsUrl").textContent = `${location.origin}/map?obs=1`;
  if (matchMedia("(hover: none)").matches) $("keys").classList.add("hidden");

  renderCamButton(); render3d(); renderSwitches(); renderStyleList(); renderStats(); renderHud();
  setStatus("connecting");
  poll();
})();
</script>
</body>
</html>
"""



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
    global rec, fog
    rec = Recorder()
    fog = Fog()
    atexit.register(lambda: (rec.save(force=True), fog.save(force=True)))
    try:
        http = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Http)
    except OSError as e:
        print(f"ERROR: port {HTTP_PORT} is already in use ({e}). Is the script already running?")
        return
    threading.Thread(target=http.serve_forever, daemon=True).start()
    if "--no-update-check" not in sys.argv:
        threading.Thread(target=check_update, daemon=True).start()

    line = "=" * 62
    print(line)
    print(f"  BeamNG.Earth Live GPS  v{VERSION}")
    print(line)
    ip = lan_ip()
    print(f"  Phone app address:  {ip}:{HTTP_PORT}")
    print(f"  Live map (browser): http://{ip}:{HTTP_PORT}/map")
    print(f"  Local JSON:         http://127.0.0.1:{HTTP_PORT}/")
    print(f"  Export route:       http://{ip}:{HTTP_PORT}/export.gpx  (or .kml)")
    print(f"  Relay:              game -> :{LISTEN_PORT} -> server :{SERVER_PORT}")
    print(line)
    t = rec.trip
    print(f"  Total driven:       {rec.odo_m / 1000:.2f} km")
    print(f"  Current trip:       {t['dist_m'] / 1000:.2f} km, {len(rec.points)} route points "
          f"(reset it on the map: Settings > Reset trip)")
    fs = fog.summary()
    print(f"  Explored (fog):     {fs['km2']:.2f} km2, {fs['cells']} cells (edit or reset it on the map: Settings > Fog)")
    print(f"  Data folder:        {DATA_DIR}")
    print(line)
    serve_relay()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Stopped.")
