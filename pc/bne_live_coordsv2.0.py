"""BeamNG.Earth Live GPS - PC relay.

Reads the live position of your car in BeamNG.Earth without changing the mod or the server EXE.

    Game (Lua mod) --TCP--> this relay :47614 --TCP--> BeamNG.Earth server (EXE) :47613

The mod sends a 'pose' message to its server about twice per second, which already contains the car's
real latitude/longitude and speed. This relay passes everything through unchanged and listens in.

The script also records your drive (works with the browser closed):
  - the driven route, kept across restarts until you reset it (export as GPX or KML)
  - trips: every drive is kept as its own trip (list on the map, show or delete them; a new drive never overwrites one)
  - the street and place you are on (OpenStreetMap's Nominatim; --no-places turns it off)
  - search, routes and turn-by-turn navigation on the map (OpenStreetMap Nominatim + Valhalla; --no-nav turns it off)
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
import collections
import json
import math
import os
import re
import socket
import struct
import sys
import threading
import time
import unicodedata
import urllib.error
import urllib.parse
import urllib.request
import uuid
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from xml.sax.saxutils import escape

VERSION = "2.0"

LISTEN_PORT = 47614       # the game connects here (-bngeport 47614)
SERVER_PORT = 47613       # the BeamNG.Earth server listens here (default)
HTTP_PORT = 47700         # JSON endpoint for the phone app / your own tools
if "--port" in sys.argv:  # (another port: BeamNG.Earth's Options tab, or by hand)
    try:
        _p = int(sys.argv[sys.argv.index("--port") + 1])
        if 1024 <= _p <= 65535:
            HTTP_PORT = _p
    except (IndexError, ValueError):
        pass
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE = os.path.join(SCRIPT_DIR, "live_position.json")
DATA_DIR = os.path.join(SCRIPT_DIR, "BNE_LiveGPS_data")   # route + stats, kept across restarts
TRACK_FILE = os.path.join(DATA_DIR, "track.ndjson")
STATS_FILE = os.path.join(DATA_DIR, "stats.json")
FOG_FILE = os.path.join(DATA_DIR, "fog.json")
HOME_FILE = os.path.join(DATA_DIR, "home.json")   # safe-home radius (currently unused)


def trips_dir():
    return os.path.join(DATA_DIR, "trips")   # one file per finished trip


TRIP_ID_RE = re.compile(r"[0-9a-f]{12}")
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
TRIP_MIN_M = 100.0         # a drive shorter than this is not worth an entry in the trip list
TRIP_IDLE_S = 1800.0       # no poses for 30 min: the next drive is a new trip
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
        self.trips = {}            # finished trips: id -> meta (the points stay on disk)
        self.places = {"from": None, "to": None}
        self._load()
        self._scan_trips()

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
            pl = s.get("places")
            if isinstance(pl, dict):
                self.places = {k: (str(pl[k])[:80] if pl.get(k) else None) for k in ("from", "to")}
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
                "trip": {k: round(v, 3) for k, v in self.trip.items()}, "places": self.places}
        try:
            tmp = STATS_FILE + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f)
            os.replace(tmp, STATS_FILE)
            self.dirty, self.saved_at = False, now
        except OSError:
            pass

    # ------------------------------------------------------------------ trips (a finished drive is kept)
    def _trip_file(self, tid):
        return os.path.join(trips_dir(), tid + ".ndjson")

    def _scan_trips(self):
        self.trips = {}
        try:
            names = os.listdir(trips_dir())
        except OSError:
            return
        for n in names:
            tid = n[:-7]
            if not (n.endswith(".ndjson") and TRIP_ID_RE.fullmatch(tid)):
                continue
            try:
                with open(os.path.join(trips_dir(), n), encoding="utf-8") as f:
                    meta = json.loads(f.readline()).get("meta")
                if isinstance(meta, dict) and meta.get("id") == tid:
                    self.trips[tid] = meta
            except (OSError, ValueError, AttributeError):
                pass

    def _archive(self, ended):
        """Lock held. Saves the current trip as a finished one (if it is a real drive) and starts a fresh trip."""
        t = self.trip
        if t["dist_m"] >= TRIP_MIN_M and len(self.points) >= 2:
            meta = {"id": self.track_id, "started": round(t["started"], 1), "ended": round(ended, 1),
                    "dist_m": round(t["dist_m"], 1), "moving_s": round(t["moving_s"], 1),
                    "max_ms": round(t["max_ms"], 2), "n": len(self.points),
                    "from": self.places.get("from"), "to": self.places.get("to")}
            try:
                os.makedirs(trips_dir(), exist_ok=True)
                tmp = self._trip_file(self.track_id) + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    f.write(json.dumps({"meta": meta}) + "\n")
                    for row in self.points:
                        f.write(json.dumps(row, separators=(",", ":")) + "\n")
                os.replace(tmp, self._trip_file(self.track_id))
                self.trips[self.track_id] = meta
            except OSError:
                pass
        self._begin_trip()

    def _begin_trip(self):
        """Lock held. An empty current trip. The odometer and the fog stay."""
        self.track_id = uuid.uuid4().hex[:12]
        self.points = []
        self.seg += 1
        self.open_seg = False
        self.prev_vok = 0.0
        self.trip = {"dist_m": 0.0, "moving_s": 0.0, "max_ms": 0.0, "started": time.time()}
        city = place_city()
        self.places = {"from": city, "to": city}
        self._write_header()
        self.dirty = True
        self.save(force=True)

    def new_trip(self):
        with self.lock:
            self._archive(self.last["t"] if self.last else time.time())

    reset_trip = new_trip

    def delete_trip(self, tid):
        with self.lock:
            if tid == self.track_id:
                self._begin_trip()               # the current trip: discarded, not saved
                return True
            if tid in self.trips:
                try:
                    os.remove(self._trip_file(tid))
                except OSError:
                    pass
                del self.trips[tid]
                return True
        return False

    def note_place(self, city):
        if not city:
            return
        with self.lock:
            if not self.places.get("from"):
                self.places["from"] = city
            if self.places.get("to") != city:
                self.places["to"] = city
                self.dirty = True

    def trips_view(self):
        with self.lock:
            t = self.trip
            cur = {"id": self.track_id, "live": True, "started": t["started"],
                   "ended": self.last["t"] if self.last else t["started"], "dist_m": round(t["dist_m"], 1),
                   "moving_s": round(t["moving_s"], 1), "max_ms": round(t["max_ms"], 2), "n": len(self.points),
                   "from": self.places.get("from"), "to": self.places.get("to")}
            lst = sorted(self.trips.values(), key=lambda m: m.get("started", 0), reverse=True)
            return {"current": cur, "trips": lst}

    def trip_rows(self, tid):
        """[[lat, lon, t, speed, seg], ...] of the current or a finished trip; None if there is no such trip."""
        with self.lock:
            if tid == self.track_id:
                return [list(p) for p in self.points], {"started": self.trip["started"], "dist_m": self.trip["dist_m"],
                                                       "max_ms": self.trip["max_ms"]}
            meta = self.trips.get(tid)
        if meta is None:
            return None
        rows = []
        try:
            with open(self._trip_file(tid), encoding="utf-8") as f:
                f.readline()
                for line in f:
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(row, list) and len(row) >= 5:
                        rows.append(row[:5])
        except OSError:
            return None
        return rows, meta

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
                    if self.trip["dist_m"] == 0.0:
                        self.trip["started"] = now
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
                     "max_ms": round(t["max_ms"], 2), "started": t["started"],
                     "from": self.places.get("from"), "to": self.places.get("to")},
            "odo_m": round(self.odo_m, 1),
        }

    def track_since(self, since, track_id):
        with self.lock:
            if track_id != self.track_id or since < 0 or since > len(self.points):
                since = 0
            pts = [[p[0], p[1], p[4]] for p in self.points[since:]]
            return {"id": self.track_id, "from": since, "n": len(self.points), "seg": self.seg, "pts": pts}

    def segments(self, rows=None):
        with self.lock:
            segs, cur, cur_id = [], [], None
            for p in (self.points if rows is None else rows):
                if p[4] != cur_id:
                    if len(cur) > 1:
                        segs.append(cur)
                    cur, cur_id = [], p[4]
                cur.append(p)
            if len(cur) > 1:
                segs.append(cur)
            return segs

    def gpx(self, rows=None):
        out = ['<?xml version="1.0" encoding="UTF-8"?>',
               '<gpx version="1.1" creator="BeamNG.Earth Live GPS by Leopri2008" '
               'xmlns="http://www.topografix.com/GPX/1/1">',
               f'<metadata><name>BeamNG.Earth drive</name><time>{iso(time.time())}</time></metadata>',
               '<trk><name>BeamNG.Earth drive</name>']
        for seg in self.segments(rows):
            out.append("<trkseg>")
            for la, lo, t, v, _ in seg:
                out.append(f'<trkpt lat="{la:.7f}" lon="{lo:.7f}"><time>{iso(t)}</time></trkpt>')
            out.append("</trkseg>")
        out.append("</trk></gpx>")
        return "\n".join(out)

    def kml(self, rows=None, meta=None):
        s = {"trip": meta} if meta else self.summary()
        desc = (f"Distance {s['trip']['dist_m'] / 1000:.1f} km, top speed {s['trip']['max_ms'] * 3.6:.0f} km/h. "
                "Recorded with BeamNG.Earth Live GPS by Leopri2008.")
        out = ['<?xml version="1.0" encoding="UTF-8"?>',
               '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>',
               '<name>BeamNG.Earth drive</name>',
               f'<description>{escape(desc)}</description>',
               '<Style id="route"><LineStyle><color>ff1a7aff</color><width>4</width></LineStyle></Style>']
        for i, seg in enumerate(self.segments(rows), 1):
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
geo = None   # street/place lookup, created in main()
home = None  # safe-home radius {lat, lon, r}
nav_enabled = True


def full_summary():
    with rec.lock:
        info = rec.summary()
    info["fog"] = fog.summary()
    info["place"] = geo.current if geo else None
    info["home"] = home
    info["nav"] = nav_enabled
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


# ===================================================================== street and place (Nominatim / OpenStreetMap)
GEO_URL = os.environ.get("BNE_GEOCODER", "https://nominatim.openstreetmap.org/reverse")
GEO_MIN_MOVE_M = 35.0      # ask again only after this much driving ...
GEO_MIN_GAP_S = 4.0        # ... and never faster than this (Nominatim's limit is 1 request per second)
GEO_CELL_DEG = 0.0003      # answers are cached per ~33 m cell, so a known road costs nothing


def parse_place(d):
    a = d.get("address") if isinstance(d, dict) else None
    if not isinstance(a, dict):
        return {}
    road = next((a[k] for k in ("road", "pedestrian", "footway", "path", "cycleway", "track") if a.get(k)), "")
    if not road and d.get("category") == "highway":
        road = d.get("name") or ""
    area = next((a[k] for k in ("suburb", "neighbourhood", "city_district", "quarter", "borough") if a.get(k)), "")
    city = next((a[k] for k in ("city", "town", "village", "municipality", "hamlet", "county") if a.get(k)), "")
    return {"road": str(road)[:80], "area": str(area)[:80], "city": str(city)[:80],
            "state": str(a.get("state") or "")[:80], "country": str(a.get("country") or "")[:80]}


def place_city():
    p = geo.current if geo else None
    return (p.get("city") or p.get("area")) if p else None


class Places(threading.Thread):
    """Looks up the road and place under the car in the background. Quiet by design: few requests, a cache,
    a polite User-Agent, and a silent retry when offline. The lookup sends the car's (game-world) coordinates."""

    def __init__(self):
        super().__init__(daemon=True)
        self.evt = threading.Event()
        self.target = None
        self.current = None
        self.cache = {}
        self.last_q = None
        self.fails = 0

    def update(self, lat, lon):
        self.target = (lat, lon)
        self.evt.set()

    def _fetch(self, la, lo):
        url = f"{GEO_URL}?format=jsonv2&zoom=17&addressdetails=1&lat={la:.6f}&lon={lo:.6f}"
        throttle("nominatim", NOMINATIM_GAP_S)
        req = urllib.request.Request(url, headers={"User-Agent": f"BeamNG.Earth-LiveGPS/{VERSION} (+https://github.com/{REPO})"})
        with urllib.request.urlopen(req, timeout=6) as r:
            return parse_place(json.loads(r.read(300000).decode("utf-8", "replace")))

    def run(self):
        while True:
            self.evt.wait()
            self.evt.clear()
            tgt = self.target
            if not tgt:
                continue
            if self.last_q and self.current is not None and dist_m(tgt[0], tgt[1], self.last_q[0], self.last_q[1]) < GEO_MIN_MOVE_M:
                continue
            key = (round(tgt[0] / GEO_CELL_DEG), round(tgt[1] / GEO_CELL_DEG))
            hit = self.cache.get(key)
            if hit is None:
                wait = GEO_MIN_GAP_S * (1 + min(self.fails, 10)) - (time.time() - (self.last_q[2] if self.last_q else 0.0))
                if wait > 0:
                    time.sleep(wait)
                tgt = self.target or tgt
                try:
                    hit = self._fetch(*tgt)
                    self.fails = 0
                    if len(self.cache) > 4000:
                        self.cache.clear()
                    self.cache[key] = hit
                except Exception:
                    self.fails += 1
                    self.last_q = (tgt[0], tgt[1], time.time())
                    continue
                self.last_q = (tgt[0], tgt[1], time.time())
            else:
                self.last_q = (tgt[0], tgt[1], self.last_q[2] if self.last_q else 0.0)
            self.current = hit or {}
            if rec and hit:
                rec.note_place(hit.get("city") or hit.get("area"))


# ===================================================================== safe-home radius
def load_home():
    global home
    try:
        with open(HOME_FILE, encoding="utf-8") as f:
            j = json.load(f)
        home = clean_home(j)
    except (OSError, ValueError, TypeError):
        home = None


def clean_home(j):
    lat, lon, r = float(j["lat"]), float(j["lon"]), float(j["r"])
    if not (-90 <= lat <= 90 and -180 <= lon <= 180) or r != r:
        raise ValueError("home")
    return {"lat": round(lat, 6), "lon": round(lon, 6), "r": int(min(1000, max(20, r)))}


def set_home(j):
    """j = {lat, lon, r} or None (remove). Raises ValueError on bad input."""
    global home
    new = None if j is None else clean_home(j)
    try:
        if new is None:
            if os.path.exists(HOME_FILE):
                os.remove(HOME_FILE)
        else:
            os.makedirs(DATA_DIR, exist_ok=True)
            with open(HOME_FILE, "w", encoding="utf-8") as f:
                json.dump(new, f)
    except OSError:
        pass
    home = new


def at_home(lat, lon):
    return bool(home) and dist_m(home["lat"], home["lon"], lat, lon) <= home["r"]


# ===================================================================== settings kept by the script (units)
def settings_file():
    return os.path.join(DATA_DIR, "settings.json")


settings = {"imperial": False}
settings_text = {}


def load_settings():
    try:
        with open(settings_file(), encoding="utf-8") as f:
            j = json.load(f)
        for k in settings:
            if isinstance(j.get(k), bool):
                settings[k] = j[k]
    except (OSError, ValueError, AttributeError):
        pass


def save_settings():
    try:
        os.makedirs(DATA_DIR, exist_ok=True)
        with open(settings_file(), "w", encoding="utf-8") as f:
            json.dump(dict(settings, **settings_text), f)
    except OSError:
        pass


# ===================================================================== search and routes (public OpenStreetMap services)
# Everything goes through the script, never straight from the browser: one polite User-Agent, a pause between requests,
# a cache for searches, and one place to switch it off (--no-nav). Needs the internet; sends your queries and the
# game-world coordinates to Nominatim (search) and to the Valhalla server (routes).
NOMINATIM_SEARCH = os.environ.get("BNE_SEARCH", "https://nominatim.openstreetmap.org/search")
ROUTER_URL = os.environ.get("BNE_ROUTER", "https://valhalla1.openstreetmap.de/route")   # or your own: --router URL
NOMINATIM_GAP_S = 1.1      # Nominatim: at most 1 request per second for everything (names, search)
ROUTER_GAP_S = 1.0
ROUTE_MAX_POINTS = 11      # the car + up to 10 stops
ROUTE_MAX_SHAPE = 150000
SEARCH_CATS = {"fuel": "fuel", "food": "restaurant", "cafe": "cafe", "parking": "parking", "charging": "charging station",
               "pharmacy": "pharmacy", "hospital": "hospital", "hotel": "hotel", "shop": "supermarket", "atm": "atm",
               "bar": "bar", "wc": "toilets", "doctor": "doctors", "sight": "attraction", "bakery": "bakery", "fast": "fast food"}
NEAR_SCALE_M = 150000.0    # a hit this far away counts half as much as the same hit next to the car
LANG_RE = re.compile(r"^[A-Za-z]{2,3}(-[A-Za-z0-9]{2,8}){0,2}$")
_gate = {"nominatim": threading.Lock(), "router": threading.Lock()}
_gate_t = {"nominatim": 0.0, "router": 0.0}


class ServiceError(Exception):
    pass


def throttle(kind, gap):
    with _gate[kind]:
        wait = gap - (time.time() - _gate_t[kind])
        if wait > 0:
            time.sleep(wait)
        _gate_t[kind] = time.time()


def user_agent():
    return f"BeamNG.Earth-LiveGPS/{VERSION} (+https://github.com/{REPO})"


class DiskCache:
    """Small answer cache, kept in a file so the same search is free tomorrow too."""

    def __init__(self, name, max_items=300, ttl=7 * 86400):
        self.name, self.max, self.ttl, self.items, self.n, self.loaded = name, max_items, ttl, {}, 0, None

    def _path(self):
        return os.path.join(DATA_DIR, self.name)

    def _load(self):
        if self.loaded == self._path():
            return
        self.loaded = self._path()
        try:
            with open(self.loaded, encoding="utf-8") as f:
                j = json.load(f)
            self.items = j if isinstance(j, dict) else {}
        except (OSError, ValueError):
            self.items = {}

    def get(self, key):
        self._load()
        e = self.items.get(key)
        if isinstance(e, dict) and time.time() - e.get("t", 0) < self.ttl:
            return e.get("v")
        return None

    def put(self, key, value):
        self._load()
        self.items[key] = {"t": time.time(), "v": value}
        if len(self.items) > self.max:
            for k in sorted(self.items, key=lambda k: self.items[k].get("t", 0))[:len(self.items) - self.max]:
                del self.items[k]
        self.n += 1
        if self.n % 5 == 0:
            self.save()

    def save(self):
        try:
            os.makedirs(DATA_DIR, exist_ok=True)
            with open(self._path(), "w", encoding="utf-8") as f:
                json.dump(self.items, f)
        except OSError:
            pass


search_cache = DiskCache("search_cache.json")


def _num(x, lo, hi):
    x = float(x)
    if not (lo <= x <= hi):
        raise ValueError("range")
    return x


def parse_hit(d, lat=None, lon=None):
    a = d.get("address") if isinstance(d.get("address"), dict) else {}
    try:
        hlat, hlon = float(d["lat"]), float(d["lon"])
    except (KeyError, ValueError, TypeError):
        return None
    name = str(d.get("name") or str(d.get("display_name") or "").split(",")[0] or "").strip()
    road = " ".join(x for x in (a.get("road"), a.get("house_number")) if x)
    city = next((a[k] for k in ("city", "town", "village", "municipality", "hamlet") if a.get(k)), "")
    addr = ", ".join(x for x in (road, a.get("suburb") or "", city) if x)
    if not name:
        name = addr.split(",")[0] if addr else "Place"
    try:
        imp = float(d.get("importance") or 0.0)
    except (ValueError, TypeError):
        imp = 0.0
    hit = {"name": name[:100], "addr": addr[:140], "lat": round(hlat, 6), "lon": round(hlon, 6),
           "type": str(d.get("type") or "")[:30], "cat": str(d.get("category") or "")[:30], "imp": imp}
    if lat is not None and lon is not None:
        hit["dist_m"] = round(dist_m(lat, lon, hlat, hlon))
    return hit


def _fold(t):
    t = unicodedata.normalize("NFKD", str(t).lower().replace("ß", "ss"))
    return "".join(c for c in t if not unicodedata.combining(c))


def _nominatim(params, ckey):
    """One Nominatim request: cached for a week, spaced by the gate, always the same polite User-Agent."""
    raw = search_cache.get(ckey)
    if raw is not None:
        return raw
    throttle("nominatim", NOMINATIM_GAP_S)
    req = urllib.request.Request(NOMINATIM_SEARCH + "?" + urllib.parse.urlencode(params), headers={"User-Agent": user_agent()})
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            raw = json.loads(r.read(600000).decode("utf-8", "replace"))
    except Exception as e:
        raise ServiceError("search is not reachable") from e
    if not isinstance(raw, list):
        raise ServiceError("search gave no list")
    search_cache.put(ckey, raw)
    return raw


def search_places(q, lat=None, lon=None, cat=None, lang=None):
    """Free text search or a category near the car ("fuel", "food", ...). Cached for a week.
    Free text asks twice: once worldwide (so a city or a famous place is found wherever it is) and once inside
    ~45 km around the car (so a small local place is found). Both lists are merged and ranked by importance,
    divided by distance: "Empire Riverside" is the hotel in Hamburg and not one on another continent."""
    have = lat is not None and lon is not None
    lang = lang if lang and LANG_RE.match(lang) else ""
    base = [("format", "jsonv2"), ("addressdetails", "1"), ("dedupe", "1")]
    if lang:
        base.append(("accept-language", lang))
    if cat:
        word = SEARCH_CATS.get(cat)
        if not word or not have:
            raise ValueError("category")
        dlat = 0.12
        dlon = dlat / max(0.2, math.cos(math.radians(lat)))
        params = base + [("limit", "20"), ("q", f"[{word}]"), ("viewbox", f"{lon - dlon:.5f},{lat + dlat:.5f},{lon + dlon:.5f},{lat - dlat:.5f}"), ("bounded", "1")]
        raw = _nominatim(params, f"cat|{cat}|{round(lat, 2)}|{round(lon, 2)}|{lang}")
        hits = [h for h in (parse_hit(d, lat, lon) for d in raw if isinstance(d, dict)) if h]
        hits.sort(key=lambda h: h.get("dist_m", 0))
        for h in hits:
            h.pop("imp", None)
        return hits
    q = (q or "").strip()[:120]
    if len(q) < 2:
        raise ValueError("query")
    runs, errors = [], 0
    jobs = [("w", base + [("limit", "10"), ("q", q)], f"q|w|{q.lower()}|{lang}")]
    if have:
        dlat = 0.4
        dlon = dlat / max(0.2, math.cos(math.radians(lat)))
        jobs.append(("l", base + [("limit", "10"), ("q", q), ("viewbox", f"{lon - dlon:.4f},{lat + dlat:.4f},{lon + dlon:.4f},{lat - dlat:.4f}"), ("bounded", "1")],
                     f"q|l|{q.lower()}|{lang}|{round(lat, 1)}|{round(lon, 1)}"))
    for _, params, ckey in jobs:
        try:
            runs.append(_nominatim(params, ckey))
        except ServiceError:
            errors += 1
    if errors == len(jobs):
        raise ServiceError("search is not reachable")
    qn = _fold(q)
    toks = [t for t in re.split(r"[\s,;]+", qn) if t]
    seen, ranked = set(), []
    for raw in runs:
        for pos, d in enumerate(raw):
            h = parse_hit(d, lat, lon) if isinstance(d, dict) else None
            if not h:
                continue
            key = (_fold(h["name"]), round(h["lat"], 3), round(h["lon"], 3))
            if key in seen:
                continue
            seen.add(key)
            score = max(h.pop("imp", 0.0), 0.04) * (1.0 + 0.3 / (1 + pos))
            if h.get("dist_m") is not None:
                score /= 1.0 + h["dist_m"] / NEAR_SCALE_M
            nn = _fold(h["name"])
            if nn.startswith(qn):
                score *= 1.5
            elif toks and all(t in nn for t in toks):
                score *= 1.25
            ranked.append((score, h))
    ranked.sort(key=lambda x: -x[0])
    return [h for _, h in ranked[:12]]


def decode_poly6(s):
    out, i, lat, lon = [], 0, 0, 0
    while i < len(s):
        for which in (0, 1):
            shift = res = 0
            while True:
                b = ord(s[i]) - 63
                i += 1
                res |= (b & 0x1F) << shift
                shift += 5
                if b < 0x20:
                    break
            d = ~(res >> 1) if res & 1 else res >> 1
            if which == 0:
                lat += d
            else:
                lon += d
        out.append([lat / 1e6, lon / 1e6])
    return out


def _plan_route(body):
    """body: {points: [{lat, lon, name}, ...] (the car first, then the stops), avoid: {highways, tolls, ferries}, lang}"""
    pts = body.get("points")
    if not isinstance(pts, list) or not 2 <= len(pts) <= ROUTE_MAX_POINTS:
        raise ValueError("points")
    locs = [{"lat": _num(p["lat"], -90, 90), "lon": _num(p["lon"], -180, 180), "type": "break"} for p in pts]
    try:   # the car is moving: the route has to start in the direction it faces (no "turn around now" when a turn is not possible)
        if pts[0].get("heading") is not None:
            locs[0]["heading"] = int(_num(pts[0]["heading"], 0, 360)) % 360
            locs[0]["heading_tolerance"] = 60
    except (TypeError, ValueError):
        pass
    avoid = body.get("avoid") if isinstance(body.get("avoid"), dict) else {}
    auto = {}
    if avoid.get("highways"):
        auto["use_highways"] = 0.0
    if avoid.get("tolls"):
        auto["use_tolls"] = 0.0
    if avoid.get("ferries"):
        auto["use_ferry"] = 0.0
    lang = str(body.get("lang") or "en-US")
    if not re.fullmatch(r"[a-z]{2,3}(-[A-Za-z]{2,4})?", lang):
        lang = "en-US"
    req_obj = {"locations": locs, "costing": "auto", "costing_options": {"auto": auto},
               "directions_options": {"units": "kilometers", "language": lang}}
    throttle("router", ROUTER_GAP_S)
    req = urllib.request.Request(ROUTER_URL, data=json.dumps(req_obj).encode(), method="POST",
                                 headers={"User-Agent": user_agent(), "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            data = json.loads(r.read(40_000_000).decode("utf-8", "replace"))
    except urllib.error.HTTPError as e:
        try:
            msg = json.loads(e.read(20000)).get("error") or ""
        except Exception:
            msg = ""
        raise ServiceError(f"no route ({msg or 'HTTP ' + str(e.code)})") from e
    except Exception as e:
        raise ServiceError("routing is not reachable") from e
    try:
        trip = data["trip"]
        shape, mans, ends = [], [], []
        for leg in trip["legs"]:
            pl = decode_poly6(leg["shape"])
            off = len(shape) - 1 if shape else 0
            shape.extend(pl[1:] if shape else pl)
            for m in leg.get("maneuvers", []):
                mans.append({"text": str(m.get("instruction") or "")[:200],
                             "say": str(m.get("verbal_pre_transition_instruction") or m.get("instruction") or "")[:240],
                             "alert": str(m.get("verbal_transition_alert_instruction") or "")[:200],
                             "dist_m": round(float(m.get("length", 0)) * 1000), "time_s": round(float(m.get("time", 0))),
                             "idx": off + int(m.get("begin_shape_index", 0)), "type": int(m.get("type", 0)),
                             "street": ", ".join(str(x) for x in (m.get("street_names") or [])[:2])[:100]})
            ends.append(len(shape) - 1)
        if len(shape) > ROUTE_MAX_SHAPE:
            raise ServiceError("that route is too long")
        sm = trip["summary"]
        return {"dist_m": round(float(sm["length"]) * 1000), "time_s": round(float(sm["time"])),
                "shape": [[round(a, 5), round(b, 5)] for a, b in shape], "maneuvers": mans, "ends": ends}
    except ServiceError:
        raise
    except (KeyError, ValueError, TypeError, IndexError) as e:
        raise ServiceError("routing gave an unexpected answer") from e


# ===================================================================== places along the route, place details, fuel prices
# Overpass (OpenStreetMap data, the same data as the map) finds places in a corridor around the route. Fuel prices come
# from national open data: Germany (Tankerkoenig, needs YOUR free key), Austria (E-Control), France, Spain. Keys stay in
# a file on this PC (BNE_LiveGPS_data/keys.json) or in an environment variable and are never sent to the browser.
OVERPASS_URL = os.environ.get("BNE_OVERPASS", "https://overpass-api.de/api/interpreter")
OVERPASS_GAP_S = 2.0
for _k in ("overpass", "fuel", "wiki"):
    _gate[_k], _gate_t[_k] = threading.Lock(), 0.0
ALONG_SEL = {
    "fuel": ['["amenity"="fuel"]'], "charging": ['["amenity"="charging_station"]'],
    "food": ['["amenity"~"^(restaurant|fast_food|food_court)$"]'], "fast": ['["amenity"="fast_food"]'],
    "cafe": ['["amenity"~"^(cafe|ice_cream)$"]'],
    "parking": ['["amenity"="parking"]["access"!~"^(private|customers|permit)$"]', '["highway"="rest_area"]'],
    "shop": ['["shop"~"^(supermarket|convenience|mall)$"]'], "pharmacy": ['["amenity"="pharmacy"]'],
    "hospital": ['["amenity"~"^(hospital|clinic)$"]'], "hotel": ['["tourism"~"^(hotel|motel|hostel|guest_house)$"]'],
    "atm": ['["amenity"~"^(atm|bank)$"]'], "bar": ['["amenity"~"^(bar|pub|biergarten)$"]'],
    "wc": ['["amenity"="toilets"]'], "doctor": ['["amenity"~"^(doctors|dentist)$"]'],
    "sight": ['["tourism"~"^(attraction|viewpoint|museum)$"]'],
}
OSM_KEEP = ("brand", "operator", "opening_hours", "phone", "contact:phone", "website", "contact:website", "cuisine", "wheelchair",
            "capacity", "fee", "drive_through", "takeaway", "internet_access", "wikipedia", "wikimedia_commons", "image",
            "addr:street", "addr:housenumber", "addr:city", "addr:postcode", "payment:cards", "toilets", "shop", "self_service",
            "opening_hours:kitchen", "socket:type2", "socket:ccs", "authentication:app")
_along_cache = {}


OVERPASS_MIRRORS = ["https://overpass.private.coffee/api/interpreter", "https://ovp.dresel.systems/api/interpreter"]
_op_bad = {}      # server -> skipped until this time (it was busy or unreachable)


def overpass(query, budget=28.0):
    """Asks one public Overpass server after the other (the main one is often overloaded); gives up after `budget` seconds."""
    t0 = time.time()
    urls = [OVERPASS_URL] + [u for u in OVERPASS_MIRRORS if u != OVERPASS_URL]
    urls = [u for u in urls if _op_bad.get(u, 0) < t0] or urls
    for u in urls:
        left = budget - (time.time() - t0)
        if left < 4:
            break
        throttle("overpass", 0.4)
        try:
            req = urllib.request.Request(u, data=urllib.parse.urlencode({"data": query}).encode(), method="POST",
                                         headers={"User-Agent": user_agent(), "Content-Type": "application/x-www-form-urlencoded"})
            with urllib.request.urlopen(req, timeout=min(left, 14)) as r:
                data = json.loads(r.read(12_000_000).decode("utf-8", "replace"))
            if not data.get("elements") and "runtime error" in str(data.get("remark", "")):
                raise OSError("query timed out on the server")
            return data
        except urllib.error.HTTPError as e:
            _op_bad[u] = time.time() + (60 if e.code in (429, 502, 503, 504) else 30)
        except Exception:
            _op_bad[u] = time.time() + 45
    raise ServiceError("the map-data servers (Overpass) are busy or not reachable right now")


def hit_from_osm(el):
    t = el.get("tags") or {}
    c = el.get("center") or {}
    lat = el.get("lat", c.get("lat"))
    lon = el.get("lon", c.get("lon"))
    if lat is None or lon is None:
        return None
    typ = t.get("amenity") or t.get("tourism") or t.get("shop") or t.get("highway") or ""
    name = t.get("name") or t.get("brand") or t.get("operator") or ""
    street = " ".join(x for x in (t.get("addr:street"), t.get("addr:housenumber")) if x)
    addr = ", ".join(x for x in (street, t.get("addr:city")) if x)
    tags = {k: str(t[k])[:240] for k in OSM_KEEP if k in t}
    fuels = [k[5:] for k, v in t.items() if k.startswith("fuel:") and v == "yes"][:12]
    if fuels:
        tags["fuels"] = ",".join(fuels)
    return {"name": name[:120], "lat": round(float(lat), 6), "lon": round(float(lon), 6), "type": typ, "addr": addr[:140],
            "tags": tags}


def along_search(body):
    """body: {points: [[lat, lon], ...] the route ahead (thinned), cat, radius (m)}"""
    cat = str(body.get("cat") or "")
    sels = ALONG_SEL.get(cat)
    pts = body.get("points")
    if not sels or not isinstance(pts, list) or not 2 <= len(pts) <= 150:
        raise ValueError("along")
    poly = ",".join(f"{_num(p[0], -90, 90):.5f},{_num(p[1], -180, 180):.5f}" for p in pts)
    rad = int(_num(body.get("radius", 1200), 200, 3000))
    ck = (cat, rad, hash(poly))
    got = _along_cache.get(ck)
    if got and time.time() - got[0] < 600:
        return got[1]
    q = "[out:json][timeout:20];(" + "".join(f"nwr{s}(around:{rad},{poly});" for s in sels) + ");out center tags 200;"
    hits = [h for h in (hit_from_osm(e) for e in overpass(q, 24).get("elements", [])) if h]
    if len(_along_cache) > 40:
        _along_cache.clear()
    _along_cache[ck] = (time.time(), hits)
    return hits


def _get_json(url, timeout=10):
    throttle("wiki", 0.3)
    req = urllib.request.Request(url, headers={"User-Agent": user_agent(), "Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read(2_000_000).decode("utf-8", "replace"))


def _wiki_host_ok(u):
    try:
        p = urllib.parse.urlparse(u)
    except ValueError:
        return False
    return p.scheme == "https" and (p.hostname or "").endswith(("wikimedia.org", "wikipedia.org"))


def place_media(tags):
    """A photo and a short text from Wikipedia / Wikimedia Commons. The text is always English: a page tagged in another
    language (nl:, de:, ...) is followed to its English article; without one, only the photo is used."""
    out = {}
    w = tags.get("wikipedia") or ""
    if ":" in w:
        lg, title = w.split(":", 1)
        if re.fullmatch(r"[a-z]{2,8}(-[a-z]{2,8})?", lg):
            try:
                if lg != "en":
                    j = _get_json(f"https://{lg}.wikipedia.org/w/api.php?action=query&format=json&prop=langlinks&lllang=en&redirects=1&titles=" + urllib.parse.quote(title, safe=""))
                    en = None
                    for pg in ((j.get("query") or {}).get("pages") or {}).values():
                        for ll in pg.get("langlinks") or []:
                            en = ll.get("*") or ll.get("title")
                    if en:
                        lg, title = "en", str(en)
                    else:
                        lg = ""
                if lg == "en":
                    j = _get_json("https://en.wikipedia.org/api/rest_v1/page/summary/" + urllib.parse.quote(title.replace(" ", "_"), safe=""))
                    th = (j.get("thumbnail") or {}).get("source")
                    if th and _wiki_host_ok(th):
                        out["photo"] = th
                    if j.get("extract"):
                        out["about"] = str(j["extract"])[:420]
                    out["wiki"] = "https://en.wikipedia.org/wiki/" + urllib.parse.quote(title.replace(" ", "_"), safe="")
            except Exception:
                pass
    if "photo" not in out:
        com = tags.get("wikimedia_commons") or ""
        img = tags.get("image") or ""
        if com.startswith("File:"):
            out["photo"] = "https://commons.wikimedia.org/wiki/Special:FilePath/" + urllib.parse.quote(com[5:]) + "?width=640"
        elif _wiki_host_ok(img):
            out["photo"] = img
    return out


_place_miss = {}


def _fold_name(t):
    return re.sub(r"[^a-z0-9]+", "", unicodedata.normalize("NFKD", t.lower()).encode("ascii", "ignore").decode())


def _name_sim(a, b):
    fa, fb = _fold_name(a), _fold_name(b)
    if not fa or not fb:
        return 0.0
    if fa == fb:
        return 1.0
    if fa in fb or fb in fa:
        return 0.8
    ta = {_fold_name(w) for w in a.split() if len(w) > 2}
    tb = {_fold_name(w) for w in b.split() if len(w) > 2}
    return 0.5 if ta & tb else 0.0


def place_details(lat, lon, name):
    ck = f"place3|{lat:.4f}|{lon:.4f}|{name.lower()[:60]}"
    got = search_cache.get(ck)
    if got is not None:
        return got
    if time.time() - _place_miss.get(ck, 0) < 600:
        return {"found": False, "tags": {}, "addr": ""}
    q = (f"[out:json][timeout:15];(nwr[\"name\"](around:60,{lat:.6f},{lon:.6f});nwr[\"brand\"](around:60,{lat:.6f},{lon:.6f});"
         f"nwr[\"amenity\"](around:40,{lat:.6f},{lon:.6f});nwr[\"shop\"](around:40,{lat:.6f},{lon:.6f});nwr[\"tourism\"](around:40,{lat:.6f},{lon:.6f}););out center tags 40;")
    best, bs = None, 1e9
    for e in overpass(q, 20).get("elements", []):
        h = hit_from_osm(e)
        if not h:
            continue
        t = e.get("tags") or {}
        d = _dist_m(lat, lon, h["lat"], h["lon"])
        sim = max([_name_sim(name, t.get(k, "")) for k in ("name", "brand", "operator", "name:en")] + [0.0]) if name else 0.0
        if name and sim == 0.0 and d > 35:
            continue
        sc = d - 150 * sim - (20 if t.get("opening_hours") else 0)
        if sc < bs:
            best, bs = h, sc
    res = {"found": bool(best), "tags": best["tags"] if best else {}, "addr": best["addr"] if best else ""}
    res.update(place_media(res["tags"]))
    if best:
        search_cache.put(ck, res)
    else:
        _place_miss[ck] = time.time()
    return res


def _dist_m(la1, lo1, la2, lo2):
    k = 111320.0
    return math.hypot((la2 - la1) * 110540.0, (lo2 - lo1) * k * math.cos(math.radians((la1 + la2) / 2)))


# ---- fuel prices
class NeedKey(ServiceError):
    pass


FUEL_RANGE = (0.3, 6.0)      # plausible price per litre, anything else is dropped
fuel_keys = {}
_fuel_cache = {}
_es_all = {"t": 0.0, "rows": []}
_de_last = [0.0]
FUEL_REGIONS = {"DE": (47.2, 55.1, 5.8, 15.1), "AT": (46.3, 49.1, 9.5, 17.2), "FR": (41.2, 51.2, -5.3, 9.7), "ES": (35.9, 43.9, -9.5, 4.4),
                "IT": (35.4, 47.2, 6.5, 18.7), "WA": (-35.3, -13.6, 112.8, 129.1), "MX": (14.4, 32.8, -118.6, -86.6),
                "SE": (55.3, 69.1, 10.9, 24.2), "DK": (54.5, 57.8, 8.0, 15.2), "GR": (34.7, 41.8, 19.3, 29.7), "HR": (42.3, 46.6, 13.4, 19.5),
                "IS": (63.2, 66.6, -24.6, -13.4), "SI": (45.4, 46.9, 13.3, 16.7), "RO": (43.6, 48.3, 20.2, 29.7)}
FUEL_NOTE = ("Prices exist for Germany (needs your own free Tankerkoenig key), Austria, France, Spain, Italy, Western Australia, Mexico, Sweden, Denmark, Greece, Croatia, Iceland, Slovenia and Romania. "
             "Everywhere else the map shows no prices.")


def load_keys():
    try:
        with open(os.path.join(DATA_DIR, "keys.json"), encoding="utf-8") as f:
            j = json.load(f)
        if isinstance(j, dict):
            for k, v in j.items():
                if isinstance(v, str) and re.fullmatch(r"[A-Za-z0-9-]{8,80}", v.strip()):
                    fuel_keys[k] = v.strip()
    except (OSError, ValueError):
        pass
    v = os.environ.get("TANKERKOENIG_KEY", "").strip()
    if re.fullmatch(r"[A-Za-z0-9-]{8,80}", v):
        fuel_keys["tankerkoenig"] = v


def _fp(x, rng=None):
    try:
        v = float(str(x).replace(",", "."))
    except (TypeError, ValueError):
        return None
    lo, hi = rng or FUEL_RANGE
    return round(v, 3) if lo < v < hi else None


def _prices(rng=None, **kw):
    return {k: p for k, p in ((k, _fp(v, rng)) for k, v in kw.items()) if p is not None}


def _get_bytes(url, limit, timeout=60):
    req = urllib.request.Request(url, headers={"User-Agent": user_agent(), "Accept": "*/*"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.read(limit)


def _xml(data):
    """XML from a public service; documents with entity tricks are refused."""
    if b"<!ENTITY" in data or b"<!DOCTYPE" in data[:4000].upper() and b"[" in data[:4000]:
        raise ServiceError("unexpected XML")
    import xml.etree.ElementTree as ET
    return ET.fromstring(data)


def fuel_de(lat, lon, rad):
    key = fuel_keys.get("tankerkoenig")
    if not key:
        raise NeedKey("needs a key")
    if time.time() - _de_last[0] < 61:
        raise ServiceError("Tankerkoenig allows one price request per minute, try again shortly")
    _de_last[0] = time.time()
    url = ("https://creativecommons.tankerkoenig.de/json/list.php?" +
           urllib.parse.urlencode({"lat": f"{lat:.4f}", "lng": f"{lon:.4f}", "rad": min(rad, 25), "sort": "dist", "type": "all", "apikey": key}))
    j = _get_json(url, 15)
    if not j.get("ok"):
        raise ServiceError("Tankerkoenig: " + str(j.get("message") or "request failed")[:120])
    out = []
    for s in j.get("stations") or []:
        try:
            out.append({"name": str(s.get("brand") or s.get("name") or "")[:80], "lat": float(s["lat"]), "lon": float(s["lng"]),
                        "addr": ", ".join(x for x in (f"{s.get('street') or ''} {s.get('houseNumber') or ''}".strip(), s.get("place")) if x)[:120],
                        "open": s.get("isOpen") if isinstance(s.get("isOpen"), bool) else None,
                        "prices": _prices(e5=s.get("e5"), e10=s.get("e10"), diesel=s.get("diesel"))})
        except (KeyError, TypeError, ValueError):
            continue
    return out


def fuel_at(lat, lon, rad):
    by = {}
    for ft, key in (("DIE", "diesel"), ("SUP", "e5")):
        url = ("https://api.e-control.at/sprit/1.0/search/gas-stations/by-address?" +
               urllib.parse.urlencode({"latitude": f"{lat:.5f}", "longitude": f"{lon:.5f}", "fuelType": ft, "includeClosed": "false"}))
        for s in _get_json(url, 15) or []:
            try:
                loc = s["location"]
                la, lo = float(loc["latitude"]), float(loc["longitude"])
                if _dist_m(lat, lon, la, lo) > rad * 1000:
                    continue
                st = by.setdefault((round(la, 5), round(lo, 5)), {"name": str(s.get("name") or "")[:80], "lat": la, "lon": lo,
                                                                   "addr": f"{loc.get('address') or ''}, {loc.get('city') or ''}".strip(", ")[:120],
                                                                   "open": s.get("open") if isinstance(s.get("open"), bool) else None, "prices": {}})
                for p in s.get("prices") or []:
                    v = _fp(p.get("amount"))
                    if v is not None and p.get("fuelType") == ft:
                        st["prices"][key] = v
            except (KeyError, TypeError, ValueError):
                continue
    return [v for v in by.values() if v["prices"]]


def fuel_fr(lat, lon, rad):
    where = f"within_distance(geom, geom'POINT({lon:.5f} {lat:.5f})', {min(rad, 15)}km)"
    url = ("https://data.economie.gouv.fr/api/explore/v2.1/catalog/datasets/prix-des-carburants-en-france-flux-instantane-v2/records?" +
           urllib.parse.urlencode({"where": where, "limit": 100}))
    out = []
    for r in (_get_json(url, 20).get("results") or []):
        try:
            g = r.get("geom") or {}
            la, lo = float(g["lat"]), float(g["lon"])
            out.append({"name": "", "lat": la, "lon": lo, "addr": f"{r.get('adresse') or ''}, {r.get('ville') or ''}".strip(", ")[:120], "open": None,
                        "prices": _prices(e5=r.get("sp95_prix"), e10=r.get("e10_prix"), diesel=r.get("gazole_prix"),
                                          e98=r.get("sp98_prix"), e85=r.get("e85_prix"), lpg=r.get("gplc_prix"))})
        except (KeyError, TypeError, ValueError):
            continue
    return [s for s in out if s["prices"]]


def fuel_es(lat, lon, rad):
    if time.time() - _es_all["t"] > 1800 or not _es_all["rows"]:
        url = "https://sedeaplicaciones.minetur.gob.es/ServiciosRESTCarburantes/PreciosCarburantes/EstacionesTerrestres/"
        req = urllib.request.Request(url, headers={"User-Agent": user_agent(), "Accept": "application/json"})
        with urllib.request.urlopen(req, timeout=60) as r:
            j = json.loads(r.read(60_000_000).decode("utf-8", "replace"))
        rows = []
        for s in j.get("ListaEESSPrecio") or []:
            try:
                la, lo = float(str(s["Latitud"]).replace(",", ".")), float(str(s["Longitud (WGS84)"]).replace(",", "."))
            except (KeyError, ValueError):
                continue
            rows.append({"name": str(s.get("Rótulo") or "").title()[:80], "lat": la, "lon": lo,
                         "addr": f"{(s.get('Dirección') or '').title()}, {(s.get('Localidad') or '').title()}".strip(", ")[:120], "open": None,
                         "prices": _prices(e5=s.get("Precio Gasolina 95 E5"), e10=s.get("Precio Gasolina 95 E10"), diesel=s.get("Precio Gasoleo A"),
                                           e98=s.get("Precio Gasolina 98 E5"), lpg=s.get("Precio Gases licuados del petróleo"))})
        _es_all.update(t=time.time(), rows=[r for r in rows if r["prices"]])
    dx = rad * 1000
    return [s for s in _es_all["rows"] if abs(s["lat"] - lat) < rad / 100.0 and _dist_m(lat, lon, s["lat"], s["lon"]) <= dx][:300]


_it = {"t": 0.0, "rows": []}
IT_REG = "https://www.mimit.gov.it/images/exportCSV/anagrafica_impianti_attivi.csv"
IT_PRC = "https://www.mimit.gov.it/images/exportCSV/prezzo_alle_8.csv"


def _csv_dicts(raw):
    import csv
    lines = raw.decode("utf-8", "replace").splitlines()
    k = next((n for n, l in enumerate(lines[:5]) if "idImpianto" in l), None)
    if k is None:
        raise ServiceError("unexpected file from the Italian ministry")
    return list(csv.DictReader(lines[k:], delimiter=";"))


def fuel_it(lat, lon, rad):
    """MIMIT open data: a daily snapshot of the prices at 08:00 (not live)."""
    if time.time() - _it["t"] > 6 * 3600 or not _it["rows"]:
        reg = {}
        for r in _csv_dicts(_get_bytes(IT_REG, 20_000_000)):
            try:
                la, lo = float(str(r["Latitudine"]).replace(",", ".")), float(str(r["Longitudine"]).replace(",", "."))
            except (KeyError, ValueError):
                continue
            if 35.0 < la < 47.5 and 6.0 < lo < 19.0:
                reg[r.get("idImpianto")] = {"name": str(r.get("Bandiera") or r.get("Nome Impianto") or "")[:80], "lat": la, "lon": lo,
                                            "addr": f"{r.get('Indirizzo') or ''}, {r.get('Comune') or ''}".strip(", ")[:120], "open": None, "prices": {}, "self": {}}
        kinds = {"benzina": "e5", "gasolio": "diesel", "gpl": "lpg"}
        for r in _csv_dicts(_get_bytes(IT_PRC, 40_000_000)):
            st = reg.get(r.get("idImpianto")); k = kinds.get(str(r.get("descCarburante") or "").strip().lower())
            v = _fp(r.get("prezzo"))
            if not st or not k or v is None or v == 1.0:      # 1.000 is a placeholder some operators report
                continue
            (st["self"] if str(r.get("isSelf")) == "1" else st["prices"]).setdefault(k, v)
        rows = []
        for st in reg.values():
            st["prices"].update(st.pop("self"))               # the self-service price, when there is one
            if st["prices"]:
                rows.append(st)
        _it.update(t=time.time(), rows=rows)
    return [s for s in _it["rows"] if abs(s["lat"] - lat) < rad / 100.0 and _dist_m(lat, lon, s["lat"], s["lon"]) <= rad * 1000][:300]


def fuel_wa(lat, lon, rad):
    """FuelWatch Western Australia: one price per day per station (cents per litre in the feed)."""
    by = {}
    for pid, key in ((1, "e10"), (2, "e5"), (6, "e98"), (4, "diesel"), (5, "lpg")):
        root = _xml(_get_bytes(f"https://www.fuelwatch.wa.gov.au/fuelwatch/fuelWatchRSS?Product={pid}&Day=today", 6_000_000, 30))
        for it in root.iter("item"):
            try:
                la, lo = float(it.findtext("latitude")), float(it.findtext("longitude"))
                if _dist_m(lat, lon, la, lo) > rad * 1000:
                    continue
                v = _fp(float(it.findtext("price")) / 100.0, (0.5, 5.0))
                if v is None:
                    continue
                st = by.setdefault((round(la, 5), round(lo, 5)), {"name": str(it.findtext("brand") or it.findtext("trading-name") or "")[:80], "lat": la, "lon": lo,
                                                                   "addr": f"{it.findtext('address') or ''}, {it.findtext('location') or ''}".strip(", ")[:120], "open": None, "prices": {}, "cur": "AUD"})
                st["prices"][key] = v
            except (TypeError, ValueError):
                continue
    return list(by.values())


_mx = {"t": 0.0, "rows": []}
CRE_PLACES = "https://publicacionexterna.azurewebsites.net/publicaciones/places"
CRE_PRICES = "https://publicacionexterna.azurewebsites.net/publicaciones/prices"


def fuel_mx(lat, lon, rad):
    """CRE Mexico: all stations with Magna (regular), Premium and Diesel in pesos per litre."""
    if time.time() - _mx["t"] > 7200 or not _mx["rows"]:
        places = {}
        for pl in _xml(_get_bytes(CRE_PLACES, 30_000_000)).iter("place"):
            try:
                loc = pl.find("location")
                places[pl.get("place_id")] = {"name": str(pl.findtext("name") or "")[:80], "lat": float(loc.findtext("y")), "lon": float(loc.findtext("x")),
                                              "addr": "", "open": None, "prices": {}, "cur": "MXN"}
            except (AttributeError, TypeError, ValueError):
                continue
        for pl in _xml(_get_bytes(CRE_PRICES, 30_000_000)).iter("place"):
            st = places.get(pl.get("place_id"))
            if not st:
                continue
            for gp in pl.findall("gas_price"):
                k = {"regular": "e10", "premium": "e5", "diesel": "diesel"}.get(gp.get("type"))
                v = _fp(gp.text, (5.0, 60.0))
                if k and v is not None:
                    st["prices"][k] = v
        _mx.update(t=time.time(), rows=[x for x in places.values() if x["prices"]])
    return [s for s in _mx["rows"] if abs(s["lat"] - lat) < rad / 100.0 and _dist_m(lat, lon, s["lat"], s["lon"]) <= rad * 1000][:300]


FEU_CODES = {"diesel": "diesel", "sp95": "e5", "e5": "e5", "e10": "e10", "sp98": "e98", "gpl": "lpg", "e85": "e85"}
FEU_COUNTRIES = ("SE", "DK", "GR", "HR", "IS", "SI", "RO", "AT", "FR", "ES", "IT")   # the last four: only as a fallback


def fuel_feu(cc):
    """fuel-prices.eu: one open interface (no key today) with station prices for many European countries, EUR per litre."""
    def fn(lat, lon, rad):
        url = "https://www.fuel-prices.eu/live/api.php?" + urllib.parse.urlencode(
            {"action": "stations", "country": cc, "lat": f"{lat:.5f}", "lng": f"{lon:.5f}", "radius": str(int(min(max(rad, 2), 50))), "limit": "100"})
        j = _get_json(url, 15)
        if not isinstance(j, dict) or not j.get("ok"):
            raise ServiceError("fuel-prices.eu did not answer")
        out = []
        for s in (j.get("data") or {}).get("stations") or []:
            try:
                la, lo = float(s["lat"]), float(s["lng"])
                pr = {}
                for code, v in (s.get("prices") or {}).items():
                    k = FEU_CODES.get(code)
                    x = _fp(v.get("price") if isinstance(v, dict) else v)
                    if k and x is not None:
                        pr[k] = x
                if pr:
                    out.append({"name": str(s.get("brand") or s.get("name") or "")[:80], "lat": la, "lon": lo,
                                "addr": ", ".join(x for x in (str(s.get("address") or ""), str(s.get("city") or "")) if x)[:120],
                                "open": s.get("is_open") if isinstance(s.get("is_open"), bool) else None, "prices": pr})
            except (KeyError, TypeError, ValueError, AttributeError):
                continue
        return out
    return fn


FUEL_PROVIDERS = {"DE": fuel_de, "AT": fuel_at, "FR": fuel_fr, "ES": fuel_es, "IT": fuel_it, "WA": fuel_wa, "MX": fuel_mx, **{c: fuel_feu(c) for c in ("SE", "DK", "GR", "HR", "IS", "SI", "RO")}}
FUEL_BACKUP = {c: fuel_feu(c) for c in ("AT", "FR", "ES", "IT")}


def fuel_prices(lat, lon, rad_km=12.0):
    rad = max(2.0, min(rad_km, 25.0))
    regs = [c for c, (a, b, c1, d) in FUEL_REGIONS.items() if a <= lat <= b and c1 <= lon <= d]
    res = {"regions": regs, "supported": bool(regs), "needs_key": [], "stations": [], "errors": [], "note": FUEL_NOTE}
    for c in regs:
        ck = (c, round(lat / 0.05), round(lon / 0.05), round(rad))
        got = _fuel_cache.get(ck)
        if got and time.time() - got[0] < (300 if c != "ES" else 600):
            res["stations"] += got[1]
            continue
        st, err = None, ""
        try:
            st = FUEL_PROVIDERS[c](lat, lon, rad)
        except NeedKey:
            res["needs_key"].append(c)
            continue
        except ServiceError as e:
            err = str(e)
        except Exception:
            err = "the price service is not reachable"
        if st is None and c in FUEL_BACKUP:                       # the national service is down: the open mirror has the same stations
            try:
                st = FUEL_BACKUP[c](lat, lon, rad)
            except Exception:
                st = None
        if st is None and got:                                    # still nothing: older prices are better than none
            res["stations"] += got[1]
            res["errors"].append(f"{c}: {err or 'not reachable'} (showing prices from {int((time.time() - got[0]) / 60)} min ago)")
            continue
        if st is None:
            res["errors"].append(f"{c}: {err}")
            continue
        for s in st:
            s["src"] = c
            s.setdefault("cur", "EUR")
        if len(_fuel_cache) > 60:
            _fuel_cache.clear()
        _fuel_cache[ck] = (time.time(), st)
        res["stations"] += st
    return res




def plan_route(body):
    try:
        return _plan_route(body)
    except ServiceError as e:
        pts = body.get("points")
        if str(e).startswith("no route") and isinstance(pts, list) and pts and isinstance(pts[0], dict) and pts[0].get("heading") is not None:
            pts = [{k: v for k, v in pts[0].items() if k != "heading"}] + pts[1:]
            return _plan_route(dict(body, points=pts))      # the direction made it impossible: plan without it
        raise


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
    if geo:
        geo.update(msg["lat"], msg["lon"])
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


FEED_URL = os.environ.get("BNE_FEED", "http://127.0.0.1:47615")   # BeamNG.Earth's companion feed (--feed)


def serve_feed(url=None):
    """--feed: the car's position from BeamNG.Earth's own companion feed (its server, 127.0.0.1:47615/v1/stream,
    read-only server-sent events, about two poses a second) instead of the relay: no -bngeport launch option, and
    nothing sits between the game and its server. Reconnects by itself."""
    import http.client
    import urllib.parse
    u = urllib.parse.urlsplit(url or FEED_URL)
    host, port = u.hostname or "127.0.0.1", u.port or 47615
    print(f"  Reading the position from BeamNG.Earth's feed ({host}:{port})...")
    said = False
    while True:
        conn = None
        try:
            conn = http.client.HTTPConnection(host, port, timeout=30)
            conn.request("GET", "/v1/stream", headers={"Accept": "text/event-stream"})
            r = conn.getresponse()
            if r.status != 200:
                raise OSError(f"HTTP {r.status}")
            if not said:
                print("\n  Connected to BeamNG.Earth. Load into the world and drive.", flush=True)
                said = True
            while True:
                line = r.fp.readline(1 << 16)
                if not line:
                    raise OSError("the feed closed")
                line = line.strip()
                if not line.startswith(b"data:"):
                    continue                        # (keep-alive comments, blank lines)
                try:
                    p = json.loads(line[5:])
                except ValueError:
                    continue
                if not isinstance(p, dict) or not p.get("ok"):
                    continue
                msg = {"lat": p.get("lat"), "lon": p.get("lon"), "speed": p.get("speed"), "vx": p.get("ve"),
                       "vy": p.get("vn"), "anchorId": p.get("anchorId")}
                try:
                    handle_pose(msg)
                except Exception as e:              # never let a bad pose stop the feed
                    print(f"\n  [warn] pose skipped: {e}", flush=True)
        except (OSError, http.client.HTTPException) as e:
            if said:
                print(f"\n  BeamNG.Earth's feed went away ({e}); trying again...", flush=True)
                said = False
            time.sleep(2.0)
        finally:
            if conn is not None:
                conn.close()


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
        elif path == "/search":
            q = {k: urllib.parse.unquote_plus(v) for k, v in self._q().items()}
            if not nav_enabled:
                self._json(403, {"error": "Search is switched off (--no-nav)"})
                return
            try:
                lat = _num(q["lat"], -90, 90) if q.get("lat") else None
                lon = _num(q["lon"], -180, 180) if q.get("lon") else None
                hits = search_places(q.get("q"), lat, lon, q.get("cat") or None, q.get("lang") or None)
            except (ValueError, TypeError):
                self._json(400, {"error": "Bad search"})
                return
            except ServiceError as e:
                self._json(502, {"error": str(e)})
                return
            self._json(200, {"hits": hits})
        elif path == "/fuel":
            q = {k: urllib.parse.unquote_plus(v) for k, v in self._q().items()}
            if not nav_enabled:
                self._json(403, {"error": "Search is switched off (--no-nav)"})
                return
            try:
                self._json(200, fuel_prices(_num(q["lat"], -90, 90), _num(q["lon"], -180, 180), _num(q.get("r", "12"), 1, 30)))
            except (ValueError, TypeError, KeyError):
                self._json(400, {"error": "Bad request"})
        elif path == "/place":
            q = {k: urllib.parse.unquote_plus(v) for k, v in self._q().items()}
            if not nav_enabled:
                self._json(403, {"error": "Search is switched off (--no-nav)"})
                return
            try:
                self._json(200, place_details(_num(q["lat"], -90, 90), _num(q["lon"], -180, 180), q.get("name", "")[:100]))
            except (ValueError, TypeError, KeyError):
                self._json(400, {"error": "Bad request"})
            except ServiceError as e:
                self._json(502, {"error": str(e)})
        elif path == "/trips":
            self._send(200, "application/json", json.dumps(rec.trips_view()).encode())
        elif path.startswith("/trips/"):
            tid, _, ext = path[7:].partition(".")
            got = rec.trip_rows(tid) if TRIP_ID_RE.fullmatch(tid) and ext in ("", "gpx", "kml") else None
            if got is None:
                self._send(404, "text/plain", b"No such trip")
                return
            rows, meta = got
            if ext == "":
                self._send(200, "application/json", json.dumps({"id": tid, "pts": [[r[0], r[1], r[4]] for r in rows]},
                                                               separators=(",", ":")).encode())
            else:
                body = (rec.gpx(rows) if ext == "gpx" else rec.kml(rows, meta)).encode("utf-8")
                ctype = "application/gpx+xml" if ext == "gpx" else "application/vnd.google-earth.kml+xml"
                self._send(200, ctype, body, {"Content-Disposition": f'attachment; filename="beamng-earth-trip-{tid[:6]}.{ext}"'})
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
        if path in ("/reset", "/trip/new"):
            rec.new_trip()
            print("\n  New trip started (the finished one is kept in the trip list).", flush=True)
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
        elif path in ("/route", "/along"):
            try:
                body = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length", "0")), 20_000)))
                if not isinstance(body, dict):
                    raise ValueError("body")
                if not nav_enabled:
                    self._json(403, {"error": "Routing is switched off (--no-nav)"})
                    return
                self._json(200, {"hits": along_search(body)} if path == "/along" else plan_route(body))
            except (ValueError, TypeError, AttributeError, KeyError):
                self._send(400, "text/plain", b"Bad request")
            except ServiceError as e:
                self._json(502, {"error": str(e)})
        elif path in ("/trips/delete", "/home"):
            try:
                body = json.loads(self.rfile.read(min(int(self.headers.get("Content-Length", "0")), 10_000)))
                if path == "/home":
                    set_home(None if body.get("clear") else body)
                    self._send(200, "application/json", json.dumps({"home": home}).encode())
                    return
                tid = str(body.get("id", ""))
                if not TRIP_ID_RE.fullmatch(tid):
                    raise ValueError("id")
            except (ValueError, TypeError, AttributeError, KeyError):
                self._send(400, "text/plain", b"Bad request")
                return
            ok = rec.delete_trip(tid)
            self._send(200 if ok else 404, "application/json", json.dumps({"ok": ok}).encode())
        elif path == "/fog/reset":
            fog.reset()
            print("\n  Fog of the world reset.", flush=True)
            self._send(200, "application/json", json.dumps({"epoch": fog.epoch}).encode())
        else:
            self._send(404, "text/plain", b"Not found")

    def _json(self, code, obj):
        self._send(code, "application/json", json.dumps(obj, separators=(",", ":")).encode())

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
#sheet, #tripsSheet, #navSheet {
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

/* street and place, like a navigation header */
#place {
  position: absolute; z-index: 3; left: var(--edge-l); top: calc(var(--edge-t) + 46px); pointer-events: none;
  max-width: min(380px, calc(100vw - var(--edge-l) - var(--edge-r) - var(--ctl) - 2 * var(--gap)));
  display: flex; align-items: center; gap: 10px; padding: 8px 15px 8px 11px; border-radius: 14px;
}
#place svg { width: 21px; height: 21px; flex: none; color: var(--accent); }
#place div { min-width: 0; }
#placeRoad, #placeArea { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
#placeRoad { font-weight: 700; font-size: 15px; line-height: 1.25; }
#placeArea { font-size: 12px; color: var(--muted); line-height: 1.3; }
body.plc #updPill, body.plc #awakeHint { top: calc(var(--edge-t) + 104px); }
body.plc.upd #awakeHint { top: calc(var(--edge-t) + 150px); }
body.fogedit #place { display: none; }

/* trips */
#tripsSheet .sheet-head { margin-bottom: 8px; }
.tbar { display: flex; gap: 8px; margin: 4px 0 12px; flex-wrap: wrap; }
.tbar .btn { height: 38px; padding: 0 14px; font-size: 13px; }
.tcard { border: 1px solid var(--line); border-radius: 16px; padding: 12px 14px; margin-bottom: 10px; background: rgba(255,255,255,.03); }
.tcard.live { border-color: rgba(255,122,26,.45); }
.thead { display: flex; align-items: center; gap: 8px; }
.tdot { width: 10px; height: 10px; border-radius: 50%; background: transparent; border: 1.5px solid var(--muted); flex: none; }
.tcard.shown .tdot { border-color: transparent; }
.tcard.live .tdot { background: var(--accent); border-color: var(--accent); }
.ttitle { font-size: 15px; font-weight: 700; flex: 1; min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.tbadge { font-size: 10px; letter-spacing: .12em; font-weight: 800; color: var(--accent-ink); background: var(--accent); border-radius: 6px; padding: 2px 6px; }
.troute { margin: 3px 0 0 18px; font-size: 13px; color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.tstats { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; margin: 10px 0 10px 0; }
.tstats span { font-size: 11px; color: var(--muted); display: flex; flex-direction: column; gap: 2px; }
.tstats b { font-size: 14px; color: var(--text); font-variant-numeric: tabular-nums; }
.tact { display: flex; flex-wrap: wrap; gap: 6px; }
.tact .btn, .tact a.btn { height: 34px; padding: 0 12px; font-size: 12.5px; display: inline-flex; align-items: center; text-decoration: none; color: var(--text); }
.tact .btn.on { background: var(--accent); color: var(--accent-ink); border-color: var(--accent); }
.tact .btn.warn { color: var(--bad); }
.tact .btn.armed { color: #fff; }
.tempty { color: var(--muted); font-size: 14px; text-align: center; padding: 22px 8px; }
#homeBox { display: flex; align-items: center; gap: 12px; margin: 4px 0 10px; }
#homeBox input { flex: 1; accent-color: var(--accent); }
#homeBox b { min-width: 74px; text-align: right; font-variant-numeric: tabular-nums; font-size: 14px; }

/* search, planner, navigation */
#navForm { display: flex; gap: 8px; margin: 2px 0 10px; }
#navQ { flex: 1; min-width: 0; height: 44px; border-radius: 12px; border: 1.5px solid var(--line); background: rgba(255,255,255,.05); color: var(--text); padding: 0 14px; font-size: 15px; }
#navQ:focus { outline: none; border-color: var(--accent); }
#navForm .btn.go, .nvbtn.go { background: var(--accent); color: var(--accent-ink); border-color: var(--accent); }
.chips { display: flex; gap: 8px; overflow-x: auto; padding: 2px 0 10px; scrollbar-width: none; }
.chips::-webkit-scrollbar { display: none; }
.chips button { flex: none; height: 36px; padding: 0 13px; border-radius: 999px; border: 1.5px solid var(--line); font-size: 13.5px; font-weight: 650; white-space: nowrap; }
.chips button.on { background: var(--accent); color: var(--accent-ink); border-color: var(--accent); }
.hit { display: flex; align-items: center; gap: 12px; padding: 11px 4px; border-bottom: 1px solid var(--line); cursor: pointer; width: 100%; text-align: left; }
.hit:last-child { border-bottom: 0; }
.hit .hi { width: 34px; height: 34px; border-radius: 50%; background: rgba(255,122,26,.16); color: var(--accent); display: grid; place-items: center; flex: none; font-size: 16px; }
.hit .ht { min-width: 0; flex: 1; }
.hit b { display: block; font-size: 15px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.hit span { display: block; font-size: 12.5px; color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.hit em { font-style: normal; font-size: 12.5px; color: var(--muted); flex: none; }
.nvbtns { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0 4px; }
.nvbtns .btn { height: 42px; }
.nvsum { font-size: 22px; font-weight: 750; margin: 2px 0 2px; }
.nvsum small { font-size: 13.5px; color: var(--muted); font-weight: 600; margin-left: 6px; }
.stoprow { display: flex; align-items: center; gap: 8px; padding: 8px 2px; border-bottom: 1px solid var(--line); }
.stoprow .sn { width: 24px; height: 24px; border-radius: 50%; background: #2f9bff; color: #fff; font-size: 12px; font-weight: 800; display: grid; place-items: center; flex: none; }
.stoprow .sn.end { background: #ff5c5c; }
.stoprow .st { flex: 1; min-width: 0; font-size: 14.5px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.stoprow button { width: 32px; height: 32px; border-radius: 9px; color: var(--muted); font-size: 16px; }
.stoprow button:hover { background: rgba(255,255,255,.07); color: var(--text); }
.optrow { display: flex; flex-wrap: wrap; gap: 8px; margin: 12px 0 2px; }
.optrow button { height: 34px; padding: 0 12px; border-radius: 999px; border: 1.5px solid var(--line); font-size: 13px; font-weight: 650; }
.optrow button.on { background: rgba(47,155,255,.2); border-color: #2f9bff; color: var(--text); }
.nvhint { font-size: 12.5px; color: var(--muted); margin: 8px 2px 0; }
#navBar { position: absolute; z-index: 4; left: var(--edge-l); top: calc(var(--edge-t) + 46px); right: calc(var(--edge-r) + var(--ctl) + 2 * var(--gap));
  border-radius: 16px; padding: 10px 12px; display: flex; flex-direction: column; gap: 8px; }
.nb-main { display: flex; align-items: center; gap: 12px; }
#nbIcon { width: 52px; height: 52px; border-radius: 14px; background: #2f9bff; color: #fff; font-size: 32px; font-weight: 800; display: grid; place-items: center; flex: none; }
.nb-txt { min-width: 0; }
#nbDist { font-size: 22px; font-weight: 800; display: block; line-height: 1.15; }
#nbText { font-size: 14px; color: var(--muted); display: block; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.nb-sum { display: flex; align-items: center; gap: 4px; font-size: 13px; font-weight: 650; border-top: 1px solid var(--line); padding-top: 7px; }
#nbSum { flex: 1; min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; font-variant-numeric: tabular-nums; }
.nb-sum .icon-btn { width: 36px; height: 36px; }
.nb-sum .icon-btn.off { opacity: .45; }
#nbEnd { height: 34px; padding: 0 12px; font-size: 13px; }
body.navon #place, body.navon #updPill, body.navon #awakeHint { display: none !important; }
body.fogedit #navBar { display: none; }
.mk { width: 26px; height: 26px; border-radius: 50%; background: #2f9bff; color: #fff; border: 2px solid #fff; font-size: 12px; font-weight: 800; display: grid; place-items: center; box-shadow: 0 2px 8px rgba(0,0,0,.45); }
.mk.end { background: #ff5c5c; }
.mk.pin { background: var(--accent); }
/* search panel: a bottom sheet on phones that leaves the map usable, a side panel on tablets and desktops */
#navSheet { max-height: min(50dvh, 520px); }
#navSheet .chips button { display: inline-flex; align-items: center; gap: 7px; padding-left: 6px; }
#navSheet .chips svg { width: 24px; height: 24px; flex: none; }
.hit .hi { background: none; width: 36px; height: 36px; }
.hit .hi svg { width: 36px; height: 36px; display: block; }
.rhead { display: flex; justify-content: space-between; gap: 10px; font-size: 12.5px; color: var(--muted); margin: 2px 4px 4px; }
.rhead b { color: var(--text); font-weight: 700; font-size: 13px; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
#navResults .btn { width: 100%; margin-top: 10px; }
.mk.pin { background: #ea4335; }
.hit em.pr { font-style: normal; text-align: right; line-height: 1.2; flex: none; }
.hit em.pr b { display: block; font-size: 15px; }
.hit em.pr small { color: var(--muted); font-size: 11.5px; }
.hit em.pr.cheap b { color: #34c759; }
.tag { display: inline-block; font-size: 10.5px; font-weight: 800; padding: 1px 6px; border-radius: 6px; background: rgba(52,199,89,.16); color: #34c759; margin-left: 6px; }
.pbar { display: flex; flex-wrap: wrap; gap: 6px; margin: 8px 0 2px; }
.pbar button { padding: 6px 12px; border-radius: 999px; border: 1px solid var(--line); font-size: 13px; font-weight: 650; color: var(--muted); }
.pbar button.on { background: var(--accent); color: var(--accent-ink); border-color: transparent; }
.pnote { font-size: 12px; color: var(--muted); margin: 6px 2px; line-height: 1.4; }
#placeDet img.ph { width: 100%; height: 150px; object-fit: cover; border-radius: 14px; margin: 10px 0 4px; display: block; }
.pd-about { font-size: 13.5px; line-height: 1.45; margin: 8px 2px; color: var(--text); }
.pd-row { display: flex; gap: 12px; padding: 9px 2px; border-top: 1px solid var(--line); font-size: 14px; align-items: baseline; }
.pd-row > span:first-child { color: var(--muted); width: 74px; flex: none; font-size: 12.5px; }
.pd-row a { color: var(--accent); word-break: break-all; }
.pd-open { color: #34c759; font-weight: 700; } .pd-closed { color: #ff6b6b; font-weight: 700; }
.pd-price { display: flex; justify-content: space-between; padding: 5px 2px; font-size: 14px; }
.pd-price b { font-variant-numeric: tabular-nums; }
#nbIcon svg { width: 40px; height: 40px; display: block; }
.seg.seg4 { grid-template-columns: repeat(2, 1fr); }
#btnArea { position: absolute; z-index: 6; transform: translateX(-50%); padding: 9px 16px; border-radius: 999px; font-weight: 700; font-size: 13.5px;
  background: var(--glass-strong); color: var(--text); border: 1px solid var(--line); box-shadow: 0 4px 18px rgba(0,0,0,.4); -webkit-backdrop-filter: blur(14px); backdrop-filter: blur(14px); }
#btnArea.hidden { display: none; }
@media (max-width: 759px) {
  #navSheet { bottom: var(--kb, 0px); max-height: min(50dvh, calc(var(--vvh, 100dvh) - 80px));
    box-shadow: 0 -10px 40px rgba(0,0,0,.45), 0 50vh 0 50vh var(--bg); }   /* the second shadow fills any gap down to the keyboard */
}
@media (min-width: 760px) {
  #navSheet { left: var(--edge-l); right: auto; bottom: auto; top: calc(var(--edge-t) + 46px); width: min(410px, 40vw);
    max-height: calc(100dvh - var(--edge-t) - var(--edge-b) - 60px); border: 1px solid var(--line); border-radius: 20px; padding: 14px 16px 16px; animation: pop .18s ease-out; }
  body.navon #navSheet { top: calc(var(--edge-t) + 190px); max-height: calc(100dvh - var(--edge-t) - var(--edge-b) - 204px); }
  #navBar { right: auto; width: min(430px, 40vw); }
}
body.navopen #place { display: none !important; }
</style>
</head>
<body>
<div id="map" aria-label="Live map"></div>

<header id="top">
  <div id="status" class="pill glass" data-s="connecting" role="status"><span class="dot"></span><span id="statusText">Connecting</span><span id="brand">BeamNG.Earth Live GPS</span></div>
  <button id="coords" class="pill glass" title="Copy coordinates">—</button>
</header>

<div id="place" class="glass hidden" aria-live="polite">
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 21s-6.5-5.6-6.5-11a6.5 6.5 0 0 1 13 0c0 5.4-6.5 11-6.5 11Z"/><circle cx="12" cy="10" r="2.4"/></svg>
  <div><div id="placeRoad"></div><div id="placeArea" class="hidden"></div></div>
</div>

<div id="navBar" class="glass hidden">
  <div class="nb-main"><div id="nbIcon">↑</div><div class="nb-txt"><b id="nbDist"></b><span id="nbText"></span></div></div>
  <div class="nb-sum"><span id="nbSum"></span>
    <button class="icon-btn" id="nbSearch" title="Search along the way" aria-label="Search along the way"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="10.5" cy="10.5" r="6.5"/><path d="m15.5 15.5 5 5"/></svg></button>
    <button class="icon-btn" id="nbOverview" title="Show the whole route" aria-label="Show the whole route"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="6" cy="18" r="2"/><circle cx="18" cy="6" r="2"/><path d="M8 18h6.5a3.5 3.5 0 0 0 0-7h-5a3.5 3.5 0 0 1 0-7H16"/></svg></button>
    <button class="btn warn" id="nbEnd">End</button></div>
</div>

<nav id="controls" class="glass" aria-label="Map controls">
  <button class="ctl" id="btnCam" title="Camera mode (C)" aria-label="Camera mode"></button>
  <button class="ctl" id="btn3d" title="3D buildings (B)" aria-label="3D buildings"><span class="t3d">3D</span></button>
  <button class="ctl" id="btnStyle" title="Map style (S)" aria-label="Map style">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linejoin="round"><path d="M12 3 2.5 8 12 13l9.5-5L12 3Z"/><path d="m2.5 12.5 9.5 5 9.5-5"/><path d="m2.5 16.5 9.5 5 9.5-5" opacity=".55"/></svg>
  </button>
  <button class="ctl" id="btnSearch" title="Search &amp; navigate (/)" aria-label="Search and navigate">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.9" stroke-linecap="round"><circle cx="10.5" cy="10.5" r="6.5"/><path d="m15.5 15.5 5 5"/></svg>
  </button>
  <button class="ctl" id="btnTrips" title="Trips (T)" aria-label="Trips">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><circle cx="6" cy="6" r="2.4"/><circle cx="18" cy="18" r="2.4"/><path d="M8.4 6H14a3.5 3.5 0 0 1 0 7h-4a3.5 3.5 0 0 0 0 7h5.6"/></svg>
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
    <button class="icon-btn" id="btnReset" title="Start a new trip" aria-label="Start a new trip">
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
    <div class="row"><label>Street &amp; place<small>Shows the road and town you are in</small></label><button class="switch" data-opt="place" role="switch"></button></div>
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
      <button class="btn" id="btnReset2">Start new trip</button>
    </div>
    <p class="note">Every drive is saved by the script on your PC as its own trip (also while this page is closed). A new trip starts when you tap Start new trip, after a restart of the script, or after 30 minutes without driving. Finished trips are never overwritten: see them, show them on the map or delete them with the trips button. GPX works in most GPS apps and Strava-like tools, KML opens in Google Earth.</p>
  </div>

  <div class="sec"><h3>Fog of the world</h3>
    <div class="row"><label>Fog<small>Places you have not driven through yet stay hidden in a mist</small></label><button class="switch" data-opt="fog" role="switch"></button></div>
    <p class="note" id="fogInfo"></p>
    <div class="btns">
      <button class="btn" id="btnFogEdit">Erase cells</button>
      <button class="btn warn" id="btnFogReset">Reset fog</button>
    </div>
    <p class="note">Every place you really drive through is revealed; teleports and glitches reveal nothing. Erase mode shows the grid and lets you remove cells that were revealed by mistake. Reset fog and the trips are separate.</p>
  </div>

  <div class="sec"><h3>Navigation &amp; map points</h3>
    <div class="row"><label>Avoid motorways<small>Default for new routes</small></label><button class="switch" data-opt="avoidHw" role="switch"></button></div>
    <div class="row"><label>Avoid tolls</label><button class="switch" data-opt="avoidToll" role="switch"></button></div>
    <div class="row"><label>Avoid ferries</label><button class="switch" data-opt="avoidFerry" role="switch"></button></div>
    <div id="poiRows"></div>
    <p class="note">Points of interest come from the map tiles and show from street level, with their own icons. Tap one to route to it or add it as a stop. Search and routes use OpenStreetMap's free services through the script (see the README for what is sent).</p>
  </div>

  <div class="sec"><h3>Where it works</h3>
    <p class="note">Worldwide: map, search, routes, navigation, points of interest. Depends on the region: fuel prices (Germany with your own free key, Austria, France, Spain), opening hours, phone numbers and photos (only where OpenStreetMap or Wikipedia have them). Ratings are not available.</p>
  </div>

  <div class="sec" style="display:none"><h3>Safe home</h3>
    <div class="row"><label>Safe home radius<small id="homeSmall">No home set</small></label></div>
    <div id="homeBox"><input type="range" id="homeR" min="0" max="100" step="1" value="50" aria-label="Safe home radius"><b id="homeRv">200 m</b></div>
    <div class="btns">
      <button class="btn" id="btnHomeHere">Set home here</button>
      <button class="btn" id="btnHomePick">Pick on map</button>
      <button class="btn warn" id="btnHomeClear">Remove</button>
    </div>
  </div>

  <div class="sec"><h3>Tips</h3>
    <p class="note">Open this page on any device in your Wi-Fi, including iPhone and iPad. For full screen on iPhone: Share → <b>Add to Home Screen</b>. The screen stays on while this page is open (setting below the map options). If it still turns off, tap the map once.</p>
    <p class="note">Stream overlay for OBS (browser source): <code id="obsUrl"></code></p>
    <p class="note" id="keys">Keys: <kbd>C</kbd> camera · <kbd>B</kbd> 3D · <kbd>S</kbd> style · <kbd>R</kbd> recenter · <kbd>F</kbd> fullscreen · <kbd>U</kbd> units · <kbd>T</kbd> trips</p>
  </div>
</aside>

<aside id="navSheet" class="hidden" role="dialog" aria-label="Search and navigate">
  <div class="grab"></div>
  <div class="sheet-head"><h2 id="navTitle">Search</h2>
    <button class="icon-btn" id="btnNavClose" aria-label="Close"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M6 6l12 12M18 6 6 18"/></svg></button>
  </div>
  <div id="navSearchView">
    <form id="navForm"><input id="navQ" type="search" placeholder="Search a place, address or e.g. gas station" autocomplete="off" enterkeyhint="search" aria-label="Search"><button class="btn go" type="submit">Search</button></form>
    <div class="pbar hidden" id="navScope"><button type="button" data-sc="along">Along route</button><button type="button" data-sc="near">Nearby</button></div>
    <div class="chips" id="navChips"></div>
    <div id="navResults"></div>
  </div>
  <div id="navPlaceView" class="hidden"></div>
  <div id="navPlanView" class="hidden"></div>
</aside>

<button id="btnArea" class="hidden" type="button">Search this area</button>

<aside id="tripsSheet" class="hidden" role="dialog" aria-label="Trips">
  <div class="grab"></div>
  <div class="sheet-head"><h2>Trips</h2>
    <button class="icon-btn" id="btnTripsClose" aria-label="Close"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M6 6l12 12M18 6 6 18"/></svg></button>
  </div>
  <div class="tbar"><button class="btn" id="btnTripNew">Start new trip</button><button class="btn hidden" id="btnTripsHide">Hide all on map</button></div>
  <div id="tripList"></div>
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
  const SRC_TRIPS = "bne-trips", LYR_TRIPS = "bne-trips-line", LYR_TRIPS_GLOW = "bne-trips-glow";
  const SRC_HOME = "bne-home", LYR_HOME_FILL = "bne-home-fill", LYR_HOME_LINE = "bne-home-line";
  const SRC_ROUTE = "bne-route", LYR_ROUTE_CASE = "bne-route-case", LYR_ROUTE = "bne-route-line", SRC_HITS = "bne-hits", LYR_HITS = "bne-hits-dot", SRC_MANS = "bne-mans", LYR_MANS = "bne-mans-icon", LYR_ARROWS = "bne-route-arrows";
  // ---- points of interest: one drawing per kind (colours like the big map apps), used on the map and in the lists
  const KCLR = { svc: "#8f9ce0", food: "#ff9e64", shop: "#4d97f5", health: "#f2827f", fun: "#12b5c8", civil: "#7b9eb2", out: "#4cb347", pin: "#ea4335" };
  const KIND = {
    fuel: ["svc", "pump", "Gas station"], charging: ["svc", "bolt", "Charging station"], parking: ["svc", "P", "Parking"], hotel: ["svc", "bed", "Hotel"], camp: ["svc", "tent", "Campsite"],
    bank: ["svc", "cash", "Bank"], atm: ["svc", "card", "ATM"],
    food: ["food", "fork", "Restaurant"], cafe: ["food", "cup", "Café"], bar: ["food", "glass", "Bar"],
    shop: ["shop", "bag", "Shop"], grocery: ["shop", "cart", "Supermarket"],
    hospital: ["health", "H", "Hospital"], pharmacy: ["health", "plus", "Pharmacy"], doctor: ["health", "plus", "Doctor"],
    post: ["civil", "mail", "Post office"], police: ["civil", "shield", "Police"], fire: ["civil", "flame", "Fire station"], wc: ["civil", "wc", "Toilets"],
    museum: ["fun", "museum", "Museum"], sight: ["fun", "camera", "Sight"], film: ["fun", "film", "Cinema / theatre"], tree: ["out", "tree", "Park & leisure"],
    place: ["pin", "dot", "Place"],
  };
  const GLY = {
    pump: '<path d="M6 20V6a2 2 0 0 1 2-2h5a2 2 0 0 1 2 2v14M4 20h13M8.5 8.5h4M15 9.5l3 2.5v5a1.5 1.5 0 0 0 3 0V10l-2.5-2.5"/>',
    bolt: '<path fill="#fff" stroke="none" d="M13.2 2.5 5.5 13.2h5.2l-1 8.3 7.8-11h-5.2l.9-8Z"/>',
    P: '<path stroke-width="2.6" d="M9 20V4.5h4.2a4.2 4.2 0 0 1 0 8.4H9"/>',
    bed: '<path d="M3 18.5V6M3 14.5h18v4M21 14.5V12a2.5 2.5 0 0 0-2.5-2.5H11v5"/><circle cx="7" cy="11" r="1.7"/>',
    tent: '<path d="M3 19.5 12 5l9 14.5ZM12 5v14.5"/>',
    cash: '<rect x="3" y="7" width="18" height="10" rx="1.5"/><circle cx="12" cy="12" r="2.3"/>',
    card: '<rect x="3" y="6" width="18" height="12" rx="2"/><path d="M3 10.5h18M6.5 15h3"/>',
    fork: '<path d="M7 3v6.5a2.3 2.3 0 0 0 4.6 0V3M9.3 3v18"/><path d="M17.5 21V3c-2.3 1.6-3.5 4.2-3.5 7.3 0 1.8 1.2 2.9 3.5 2.9"/>',
    cup: '<path d="M5 8.5h11v5.5a5 5 0 0 1-5 5h-1a5 5 0 0 1-5-5V8.5ZM16 10h1.3a2.4 2.4 0 0 1 0 4.8H16M4.5 21.5h12"/>',
    glass: '<path d="M5 4h14l-7 8.5L5 4ZM12 12.5V20M8 20h8"/>',
    bag: '<path d="M5.5 8h13l1 12.5h-15L5.5 8ZM9 10.5V7a3 3 0 0 1 6 0v3.5"/>',
    cart: '<path d="M3 4.5h2.6l2.1 10h9.6l2-7.5H6.4"/><circle cx="9.4" cy="19" r="1.5" fill="#fff" stroke="none"/><circle cx="16.6" cy="19" r="1.5" fill="#fff" stroke="none"/>',
    H: '<path stroke-width="2.6" d="M7 4.5v15M17 4.5v15M7 12h10"/>',
    plus: '<path stroke-width="3" d="M12 5v14M5 12h14"/>',
    mail: '<rect x="3" y="6" width="18" height="12" rx="1.5"/><path d="m3.5 7 8.5 6.5L20.5 7"/>',
    shield: '<path d="M12 3 5 6v5.5c0 4.3 3 7.8 7 9.5 4-1.7 7-5.2 7-9.5V6l-7-3Z"/>',
    flame: '<path fill="#fff" stroke="none" d="M12 2.5c1 4.2 5.5 5.6 5.5 10.6a5.5 5.5 0 0 1-11 0c0-2.3 1.1-3.6 2.3-4.6.1 2 1 3 2.2 3.3-.7-4 .2-6.4 1-9.3Z"/>',
    wc: '<circle cx="8" cy="5" r="1.8"/><path d="M8 8.5v6M5.5 21 8 14.5l2.5 6.5M14.5 9h6M17.5 9v12"/>',
    museum: '<path d="M3 9.5 12 4l9 5.5M5.5 10.5V18M10 10.5V18M14 10.5V18M18.5 10.5V18M3.5 20.5h17"/>',
    camera: '<path d="M4 8h3.5l1.5-2.5h6L16.5 8H20v11H4V8Z"/><circle cx="12" cy="13" r="3.3"/>',
    film: '<rect x="4" y="4.5" width="16" height="15" rx="1.5"/><path d="M8 4.5v15M16 4.5v15M4 9h4M4 14.5h4M16 9h4M16 14.5h4"/>',
    tree: '<path d="M12 3 6 11h3l-3.5 6h13L15 11h3L12 3ZM12 17v4"/>',
    dot: '<circle cx="12" cy="12" r="3.6" fill="#fff" stroke="none"/>',
  };
  const kindSvg = (k, pin, w) => {
    const c = KCLR[KIND[k][0]], g = '<g fill="none" stroke="#fff" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">' + GLY[KIND[k][1]] + "</g>";
    return pin
      ? `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${Math.round(w * 46 / 36)}" viewBox="0 0 36 46"><path d="M18 45C18 45 3.5 28 3.5 17.5a14.5 14.5 0 0 1 29 0C32.5 28 18 45 18 45Z" fill="${c}" stroke="#fff" stroke-width="2"/><g transform="translate(9 8.5) scale(.75)">${g}</g></svg>`
      : `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${w}" viewBox="0 0 24 24"><circle cx="12" cy="12" r="11" fill="${c}" stroke="#fff" stroke-width="1.5"/><g transform="translate(5.5 5.5) scale(.54)">${g}</g></svg>`;
  };
  // [kind, tier, classes, subclasses]: tier 0 shows from zoom 14, 1 from 15.2, 2 from 16.4 (the map tiles carry far more points than fit on a screen)
  const POI_Z = [14, 15.4, 17];
  const RANK_BANDS = [[0, 40], [15, 50], [16, 80], [17, 120], [17.8, 200], [18.6, 500]];   // zoom from, max tile rank (1 = most important): fewer, more important places when zoomed out (tuned on dense Hamburg)
  const rankCapFor = z => { let c = 40; for (const [zz, cc] of RANK_BANDS) if (z >= zz) c = cc; return Math.max(8, Math.round(c * Math.min(1, Math.max(0.4, 344000 / (innerWidth * innerHeight))))); };   // big screens show fewer per area
  const POI_BASE = {}; let rankCap = -1;
  const POI_GROUPS = [
    { id: "fuel", name: "Fuel & charging", rules: [["fuel", 0, ["fuel"], ["fuel"]], ["charging", 2, ["fuel"], ["charging_station"]]] },
    { id: "food", name: "Food & drink", rules: [["food", 1, ["restaurant"]], ["cafe", 1, ["cafe"]], ["bar", 1, ["bar", "beer"]], ["food", 2, ["fast_food", "food_court"]], ["cafe", 2, ["ice_cream"]]] },
    { id: "shop", name: "Shops", rules: [["grocery", 0, ["grocery"], ["supermarket", "department_store", "marketplace"]], ["grocery", 1, ["grocery"], ["deli", "greengrocer"]],
      ["shop", 1, ["shop"], ["mall", "convenience", "chemist", "electronics", "sports", "doityourself", "hardware", "garden_centre"]], ["shop", 2, ["bakery", "clothing_store"]]] },
    { id: "park", name: "Parking", rules: [["parking", 2, ["parking"]]] },
    { id: "stay", name: "Hotels & camping", rules: [["hotel", 0, ["lodging"], ["hotel", "hostel", "guest_house", "motel", "apartment", "chalet"]], ["camp", 1, ["campsite", "camp_site", "caravan_site"]]] },
    { id: "health", name: "Health", rules: [["hospital", 0, ["hospital"], ["hospital", "clinic"]], ["pharmacy", 1, ["pharmacy"]], ["doctor", 2, ["doctors", "dentist", "veterinary"]]] },
    { id: "service", name: "Banks, post, toilets", rules: [["bank", 1, ["bank"]], ["atm", 2, ["atm"]], ["post", 1, ["post"], ["post_office"]], ["police", 1, ["police"]], ["fire", 1, ["fire_station"]], ["wc", 2, ["toilets"]]] },
    { id: "sight", name: "Sights & parks", rules: [["museum", 0, ["museum"]], ["tree", 0, ["zoo"]], ["sight", 0, ["castle"]], ["sight", 1, ["attraction"]], ["film", 1, ["cinema", "theatre"]],
      ["sight", 2, ["monument"]], ["museum", 2, ["art_gallery"], ["gallery", "arts_centre"]], ["tree", 2, ["park", "playground", "stadium", "swimming", "golf"]]] },
  ];
  const kindOfTile = (cls, sub) => {
    for (const g of POI_GROUPS) for (const r of g.rules) if (r[2].includes(cls) && (!r[3] || r[3].includes(sub))) return r[0];
    return "place";
  };
  const OSM_KIND = { fuel: "fuel", charging_station: "charging", parking: "parking", parking_space: "parking", restaurant: "food", fast_food: "food", food_court: "food", cafe: "cafe", ice_cream: "cafe", bakery: "cafe",
    bar: "bar", pub: "bar", biergarten: "bar", nightclub: "bar", pharmacy: "pharmacy", hospital: "hospital", clinic: "hospital", doctors: "doctor", dentist: "doctor", veterinary: "doctor",
    hotel: "hotel", hostel: "hotel", guest_house: "hotel", motel: "hotel", apartment: "hotel", camp_site: "camp", caravan_site: "camp", atm: "atm", bank: "bank",
    supermarket: "grocery", marketplace: "grocery", convenience: "shop", mall: "shop", department_store: "shop", toilets: "wc", post_office: "post", police: "police", fire_station: "fire",
    museum: "museum", gallery: "museum", attraction: "sight", viewpoint: "sight", castle: "sight", monument: "sight", cinema: "film", theatre: "film",
    park: "tree", zoo: "tree", playground: "tree", stadium: "tree", swimming_pool: "tree" };
  const kindOfHit = h => h.kind || OSM_KIND[h.type] || "place";
  // search categories: what the chips and the typed words (any of many languages) stand for; "tile" = how to find them in the map tiles
  const CATS = {
    fuel: { n: "Gas stations", kind: "fuel", tile: [["fuel", ["fuel"]]] },
    charging: { n: "Charging stations", kind: "charging", tile: [["fuel", ["charging_station"]]] },
    food: { n: "Restaurants", kind: "food", tile: [["restaurant"], ["fast_food"], ["food_court"]] },
    cafe: { n: "Cafés", kind: "cafe", tile: [["cafe"], ["ice_cream"]] },
    parking: { n: "Parking", kind: "parking", tile: [["parking"]] },
    shop: { n: "Supermarkets", kind: "grocery", tile: [["grocery", ["supermarket", "department_store", "marketplace"]], ["shop", ["mall", "convenience"]]] },
    pharmacy: { n: "Pharmacies", kind: "pharmacy", tile: [["pharmacy"]] },
    hospital: { n: "Hospitals", kind: "hospital", tile: [["hospital", ["hospital", "clinic"]]] },
    hotel: { n: "Hotels", kind: "hotel", tile: [["lodging", ["hotel", "hostel", "guest_house", "motel"]]] },
    atm: { n: "ATMs & banks", kind: "atm", tile: [["atm"], ["bank"]] },
    bar: { n: "Bars & pubs", kind: "bar", tile: [["bar"], ["beer"]] },
    wc: { n: "Toilets", kind: "wc", tile: [["toilets"]] },
    doctor: { n: "Doctors", kind: "doctor", tile: [["doctors"], ["dentist"]] },
    sight: { n: "Sights", kind: "sight", tile: [["museum"], ["attraction"], ["castle"], ["monument"]] },
    bakery: { n: "Bakeries", kind: "cafe", tile: [["bakery"]] },
    fast: { n: "Fast food", kind: "food", tile: [["fast_food"]] },
  };
  const CHIPS = [["fuel", "Fuel"], ["charging", "Charging"], ["food", "Food"], ["fast", "Fast food"], ["cafe", "Café"], ["parking", "Parking"], ["shop", "Shops"], ["pharmacy", "Pharmacy"], ["hospital", "Hospital"], ["hotel", "Hotels"], ["atm", "ATM"], ["bar", "Bars"], ["wc", "Toilets"], ["sight", "Sights"]];
  const fold = t => String(t).normalize("NFD").replace(/[\u0300-\u036f]/g, "").replace(/ß/g, "ss").toLowerCase().trim();
  const TERMS = Object.entries({
    fuel: "tankstelle|tankstellen|tanken|tank|benzin|diesel|sprit|gas|gas station|petrol|petrol station|fuel|filling station|gasoline|service station|station service|station-service|essence|carburant|gasolinera|gasolina|estacion de servicio|benzina|distributore|distributore di benzina|benzinaio|stazione di servizio|tankstation|benzinestation|stacja paliw|stacja benzynowa|posto de gasolina|bensinstasjon|bensinstation|benzinlik|azs|заправка|азс|加油站|ガソリンスタンド|주유소",
    charging: "ladestation|ladestationen|ladesaeule|ladesaule|ladepunkt|laden|aufladen|e-tankstelle|charging|charging station|charger|ev|ev charging|supercharger|borne de recharge|recharge|borne|cargador|punto de recarga|colonnina|ricarica|laadpaal|ladowarka|зарядка|充电站",
    fast: "fast food|fastfood|schnellrestaurant|burger|burger king|mcdonalds|mcdonald s|mc donalds|kfc|subway|imbiss|doener|kebab|pommes|drive in|drive through|mcdrive|restauration rapide|comida rapida|tavola calda",
    food: "restaurant|restaurants|essen|essen gehen|food|eat|eating|hunger|pizza|restaurante|ristorante|trattoria|manger|nourriture|comida|cibo|mangiare|eten|jedzenie|ресторан|еда|餐厅|レストラン|식당",
    cafe: "cafe|kaffee|coffee|coffee shop|kaffeehaus|cafeteria|caffe|koffie|kawiarnia|кофе|咖啡|カフェ",
    parking: "parkplatz|parkplaetze|parkplatze|parken|parkhaus|parking|car park|garage|parcheggio|parkeren|estacionamiento|aparcamiento|stationnement|parkering|estacionamento|парковка|停车场",
    shop: "supermarkt|supermarket|einkaufen|einkauf|einkaufsladen|shopping|grocery|groceries|store|lebensmittel|supermercado|supermarche|supermercato|tienda|sklep|магазин|超市",
    pharmacy: "apotheke|apotheken|pharmacy|pharmacie|farmacia|apotheek|apteka|аптека|药店|薬局",
    hospital: "krankenhaus|klinik|hospital|notaufnahme|emergency room|er|urgences|hopital|ospedale|pronto soccorso|ziekenhuis|szpital|больница|医院",
    hotel: "hotel|hotels|unterkunft|uebernachten|ubernachten|schlafen|motel|hostel|pension|lodging|accommodation|where to stay|albergo|alojamiento|hebergement|hotell|酒店|ホテル",
    atm: "geldautomat|geldautomaten|atm|bankomat|bank|geld abheben|cash|cash machine|distributeur|cajero|bancomat|банкомат",
    bar: "bar|kneipe|pub|biergarten|drinks|cocktail|bares|ausgehen",
    wc: "toilette|toiletten|wc|klo|restroom|toilet|toilets|bathroom|servicios|bagno|toilettes",
    doctor: "arzt|aerzte|arztpraxis|doctor|doctors|zahnarzt|dentist|medecin|medico|dottore",
    sight: "sehenswuerdigkeit|sehenswuerdigkeiten|sehenswurdigkeiten|sights|sightseeing|attractions|museum|museen|tourist|tourist attractions|aussichtspunkt|viewpoint|monument",
    bakery: "baeckerei|backerei|baecker|bakery|boulangerie|panaderia|panetteria",
  }).map(([c, t]) => [c, t.split("|").map(fold)]);
  const FILLER = /\b(in der naehe|in meiner naehe|in der nahe|in meiner nahe|naechste[rnms]?|naheste[rnms]?|nahe|near me|nearby|nearest|closest|close by|around me|next|the|der|die|das|den|ein|eine|pres de moi|proche|cerca de mi|cercano|vicino a me|vicino|piu vicino|dichtbij|in de buurt)\b/g;
  const cleanQ = t => fold(t).replace(FILLER, " ").replace(/\s+/g, " ").trim();
  function matchCats(raw) {
    const q = cleanQ(raw), out = [];
    if (q.length < 2) return out;
    for (const [cat, terms] of TERMS) {
      let w = 0;
      for (const t of terms) { if (t === q) { w = 3; break; } if (q.length >= 3 && t.length > q.length && t.startsWith(q)) w = 2; }
      if (w) out.push({ cat, exact: w === 3, w });
    }
    return out.sort((a, b) => b.w - a.w);
  }
  const PLACE_RE = /^(.+?)\s+(?:in|bei|near|nahe|um|at|auf|en|dans|vicino a|nei dintorni di)\s+(.+)$/i;
  function parseQuery(raw) {
    const e = matchCats(raw).find(m => m.exact); if (e) return { cat: e.cat };
    const m = raw.match(PLACE_RE); if (!m) return null;
    const c = matchCats(m[1]).find(x => x.exact);
    return c ? { cat: c.cat, where: m[2].trim() } : null;
  }
  // rasterised icons for the map layers (the drawings above, once per kind, as a round badge and as a pin)
  const poiImgs = {}, imgDpr = Math.min(3, Math.max(2, Math.ceil(window.devicePixelRatio || 1)));
  function rasterize(svg, w, h) {
    return new Promise(res => {
      const im = new Image();
      im.onload = () => { try { const c = document.createElement("canvas"); c.width = w; c.height = h; const x = c.getContext("2d"); x.drawImage(im, 0, 0, w, h); res(x.getImageData(0, 0, w, h)); } catch (e) { res(null); } };
      im.onerror = () => res(null);
      im.src = "data:image/svg+xml;charset=utf-8," + encodeURIComponent(svg);
    });
  }
  function addKindImages() { for (const id in poiImgs) if (!map.hasImage(id)) { try { map.addImage(id, poiImgs[id], { pixelRatio: imgDpr }); } catch (e) {} } }
  async function renderKindImages() {
    await Promise.all(Object.keys(KIND).flatMap(k => [false, true].map(async pin => {
      const w = (pin ? 34 : 30) * imgDpr, h = pin ? Math.round(w * 46 / 36) : w;
      const d = await rasterize(kindSvg(k, pin, w), w, h);
      if (d) poiImgs[(pin ? "pin-" : "poi-") + k] = d;
    })));
    const tw = 34 * imgDpr;
    await Promise.all(TURN_KEYS.map(async k => { const t = { sharpR: 11, sharpL: 14, slightR: 9, slightL: 16, rightR: 10, rightL: 15, uturnR: 12, uturnL: 13, roundR: 26, roundL: 26 }[k]; const d = await rasterize(turnSvg(t, tw, true), tw, tw); if (d) poiImgs["turn-" + k] = d; }));
    const cw = 16 * imgDpr, ch = await rasterize(`<svg xmlns="http://www.w3.org/2000/svg" width="${cw}" height="${cw}" viewBox="0 0 16 16"><path d="M5 2.5 11 8 5 13.5" fill="none" stroke="#fff" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"/></svg>`, cw, cw);
    if (ch) poiImgs["route-chevron"] = ch;
    addKindImages(); try { map.triggerRepaint(); } catch (e) {}
  }

  // turn arrows: drawn by us, used in the navigation bar and as badges on the route (Valhalla maneuver types)
  const TURN_PATH = {
    straight: '<path d="M12 20V6M6.5 11.5 12 6l5.5 5.5"/>', slight: '<path d="M8 20v-5l7-7M9.5 8H16v6.5"/>',
    right: '<path d="M7 20v-7a4 4 0 0 1 4-4h8M15 5l4 4-4 4"/>', sharp: '<path d="M6 6h6a5 5 0 0 1 5 5v8M13 15.5l4 4 4-4"/>',
    uturn: '<path d="M8 20V10a4 4 0 0 1 8 0v8M12.5 14.5 16 18l3.5-3.5"/>', round: '<circle cx="12" cy="14" r="4"/><path d="M12 22v-4M12 10V3.5M8.6 7 12 3.5 15.4 7"/>',
    dest: '<path d="M7 21V4M7 5h11l-2.5 4 2.5 4H7"/>',
  };
  const TURN_OF = { 4: ["dest", 0], 5: ["dest", 0], 6: ["dest", 0], 9: ["slight", 0], 18: ["slight", 0], 20: ["slight", 0], 23: ["slight", 0], 10: ["right", 0], 11: ["sharp", 0], 12: ["uturn", 0], 13: ["uturn", 1],
    14: ["sharp", 1], 15: ["right", 1], 16: ["slight", 1], 19: ["slight", 1], 21: ["slight", 1], 24: ["slight", 1], 26: ["round", 0], 27: ["round", 0] };
  const turnOf = t => TURN_OF[t] || ["straight", 0];
  const turnKey = t => { const [g, f] = turnOf(t); return g + (f ? "L" : "R"); };
  const TURN_KEYS = ["slight", "right", "sharp", "uturn", "round"].flatMap(g => [g + "R", g + "L"]);
  const TURN_BADGE = new Set([9, 10, 11, 12, 13, 14, 15, 16, 18, 19, 20, 21, 23, 24, 26, 27]);
  const turnSvg = (t, size, badge) => {
    const [g, f] = turnOf(t), ink = `<g${f ? ' transform="translate(24 0) scale(-1 1)"' : ""} fill="none" stroke="#fff" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round">${TURN_PATH[g]}</g>`;
    return badge ? `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}" viewBox="0 0 24 24"><circle cx="12" cy="12" r="11.2" fill="#2f9bff" stroke="#fff" stroke-width="1.6"/><g transform="translate(3.4 3.4) scale(.72)">${ink}</g></svg>`
      : `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}" viewBox="0 0 24 24">${ink}</svg>`;
  };
  const params = new URLSearchParams(location.search);
  const OBS = params.get("obs") === "1";

  // ---------------------------------------------------------------- settings (per browser)
  const DEFAULTS = { style: "dark", cam: "chase", buildings: true, autoZoom: true, trail: true, stats: true, units: "metric", awake: true, fog: false, place: true, avoidHw: false, avoidToll: false, avoidFerry: false, fuelType: "e10",
    poi_fuel: false, poi_food: false, poi_shop: false, poi_park: false, poi_stay: false, poi_health: false, poi_service: false, poi_sight: false };
  const store = {
    load() { try { return JSON.parse(localStorage.getItem("bne-map") || "{}"); } catch (e) { return {}; } },
    save(s) { try { localStorage.setItem("bne-map", JSON.stringify(s)); } catch (e) {} },
  };
  const S = Object.assign({}, DEFAULTS, OBS ? {} : store.load());
  // URL overrides (handy for OBS): ?style=liberty&cam=heading&units=imperial&buildings=0&trail=0&stats=1&fog=0&place=0
  for (const k of ["style", "cam", "units"]) if (params.get(k)) S[k] = params.get(k);
  for (const k of ["buildings", "autoZoom", "trail", "stats", "fog", "place"]) if (params.has(k)) S[k] = params.get(k) !== "0";
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

  // a new trip only closes the current one (it is saved under Trips), so one tap is enough
  async function resetTrip() {
    try {
      const r = await fetch("/trip/new", { method: "POST" });
      applySummary(await r.json());
      trail.segs = []; trail.segIds = []; trail.n = 0; trail.dirty = true;
      await loadTrack(true);
      toast("New trip started. The last one is saved under Trips");
      if (!$("tripsSheet").classList.contains("hidden")) loadTrips();
    } catch (e) { toast("PC script not reachable"); }
  }
  function applySummary(j) {
    if (!j) return;
    if (j.trip) { trip.dist = +j.trip.dist_m || 0; trip.moving = +j.trip.moving_s || 0; trip.max = +j.trip.max_ms || 0; }
    if (typeof j.odo_m === "number") trip.odo = j.odo_m;
    if (j.track_id && (j.track_id !== trail.id || j.track_n > trail.n)) loadTrack(j.track_id !== trail.id);
    else if (j.track_id && j.track_n < trail.n) loadTrack(true);
    if (j.fog) onFogInfo(j.fog);
    if (j.update) showUpdate(j.update);
    if (j.place !== undefined) { lastPlace = j.place; showPlace(lastPlace); }
    if (j.home !== undefined) onHome(j.home);
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
    if (nav.mode === "drive") navTick(s.lat, s.lng, s.speed);

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
  function styleFont() {
    for (const l of (map.getStyle() && map.getStyle().layers) || []) {
      const f = l.layout && l.layout["text-font"];
      if (Array.isArray(f) && f.length && typeof f[0] === "string" && f[0] !== "literal" && f[0] !== "get") return f;
    }
    return ["Noto Sans Regular"];
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

    // finished trips shown on demand, and the safe-home circle (never in the OBS overlay)
    if (!map.getSource(SRC_TRIPS)) map.addSource(SRC_TRIPS, { type: "geojson", data: tripsGeoJSON() });
    if (!OBS && !map.getSource(SRC_HOME)) map.addSource(SRC_HOME, { type: "geojson", data: homeGeoJSON() });
    if (!OBS && !map.getLayer(LYR_HOME_FILL)) map.addLayer({ id: LYR_HOME_FILL, type: "fill", source: SRC_HOME, paint: { "fill-color": "#6cb6ff", "fill-opacity": 0.13 } }, before);
    if (!OBS && !map.getLayer(LYR_HOME_LINE)) map.addLayer({ id: LYR_HOME_LINE, type: "line", source: SRC_HOME, paint: { "line-color": "#6cb6ff", "line-width": 2, "line-dasharray": [2, 2], "line-opacity": 0.9 } }, before);
    if (!map.getLayer(LYR_TRIPS_GLOW)) map.addLayer({
      id: LYR_TRIPS_GLOW, type: "line", source: SRC_TRIPS, layout: { "line-cap": "round", "line-join": "round" },
      paint: { "line-color": ["get", "c"], "line-opacity": 0.25, "line-blur": 2, "line-width": ["interpolate", ["linear"], ["zoom"], 5, 4, 12, 9, 17, 20] },
    }, before);
    if (!map.getLayer(LYR_TRIPS)) map.addLayer({
      id: LYR_TRIPS, type: "line", source: SRC_TRIPS, layout: { "line-cap": "round", "line-join": "round" },
      paint: { "line-color": ["get", "c"], "line-opacity": 0.95, "line-width": ["interpolate", ["linear"], ["zoom"], 5, 1.5, 12, 3.5, 17, 7] },
    }, before);

    // planned route (blue, with a white casing), search results, points of interest by group
    if (!map.getSource(SRC_ROUTE)) map.addSource(SRC_ROUTE, { type: "geojson", data: routeGeoJSON() });
    if (!map.getSource(SRC_HITS)) map.addSource(SRC_HITS, { type: "geojson", data: hitsGeoJSON() });
    if (!map.getLayer(LYR_ROUTE_CASE)) map.addLayer({
      id: LYR_ROUTE_CASE, type: "line", source: SRC_ROUTE, layout: { "line-cap": "round", "line-join": "round" },
      paint: { "line-color": "#ffffff", "line-opacity": 0.9, "line-width": ["interpolate", ["linear"], ["zoom"], 5, 4, 12, 8, 17, 16] },
    }, before);
    if (!map.getLayer(LYR_ROUTE)) map.addLayer({
      id: LYR_ROUTE, type: "line", source: SRC_ROUTE, layout: { "line-cap": "round", "line-join": "round" },
      paint: { "line-color": "#2f9bff", "line-width": ["interpolate", ["linear"], ["zoom"], 5, 2.5, 12, 5.5, 17, 11] },
    }, before);
    if (!map.getSource(SRC_MANS)) map.addSource(SRC_MANS, { type: "geojson", data: mansGeoJSON() });
    if (!map.getLayer(LYR_ARROWS)) map.addLayer({
      id: LYR_ARROWS, type: "symbol", source: SRC_ROUTE, minzoom: 12,
      layout: { "symbol-placement": "line", "symbol-spacing": 70, "icon-image": "route-chevron", "icon-size": ["interpolate", ["linear"], ["zoom"], 12, 0.5, 17, 0.85],
        "icon-allow-overlap": true, "icon-ignore-placement": true, "icon-rotation-alignment": "map", "icon-padding": 0 },
    }, before);
    if (!map.getLayer(LYR_MANS)) map.addLayer({
      id: LYR_MANS, type: "symbol", source: SRC_MANS, minzoom: 13,
      layout: { "icon-image": ["concat", "turn-", ["get", "k"]], "icon-size": ["interpolate", ["linear"], ["zoom"], 13, 0.62, 17, 0.95], "icon-allow-overlap": false, "icon-padding": 6, "symbol-sort-key": ["get", "i"] },
    }, before);
    // points of interest as icons, by group and tier; search results as pins with a name
    {
      const font = styleFont(), txt = dark ? "#e8edf3" : "#2b3440", halo = dark ? "rgba(14,18,26,.92)" : "rgba(255,255,255,.95)";
      const tpaint = { "text-color": txt, "text-halo-color": halo, "text-halo-width": 1.6 };
      if (vsrc) for (const g of POI_GROUPS) for (let t = 0; t < 3; t++) {
        const rs = g.rules.filter(r => r[1] === t), id = `bne-poi-${g.id}-${t}`;
        if (!rs.length || map.getLayer(id)) continue;
        const cond = r => r[3] ? ["all", ["in", ["get", "class"], ["literal", r[2]]], ["in", ["get", "subclass"], ["literal", r[3]]]] : ["in", ["get", "class"], ["literal", r[2]]];
        const kinds = [...new Set(rs.map(r => r[0]))];
        const icon = kinds.length === 1 ? "poi-" + kinds[0] : ["case", ...rs.slice(0, -1).flatMap(r => [cond(r), "poi-" + r[0]]), "poi-" + rs[rs.length - 1][0]];
        map.addLayer({
          id, type: "symbol", source: vsrc, "source-layer": "poi", minzoom: POI_Z[t], filter: ["all", (POI_BASE[id] = ["any", ...rs.map(cond)]), ["<=", ["coalesce", ["get", "rank"], 99], rankCapFor(map.getZoom())]],
          layout: { "icon-image": icon, "icon-size": ["interpolate", ["linear"], ["zoom"], 14, 0.7, 17, 0.95], "icon-padding": ["step", ["zoom"], 12, 16, 9, 17.5, 5], "symbol-sort-key": ["coalesce", ["get", "rank"], 999],
            "text-field": ["step", ["zoom"], "", 15.6, ["coalesce", ["get", "name"], ""]], "text-font": font, "text-size": 11.5, "text-anchor": "top", "text-offset": [0, 1.05], "text-max-width": 7, "text-optional": true,
            visibility: S["poi_" + g.id] ? "visible" : "none" },
          paint: tpaint,
        });
      }
      if (!map.getLayer(LYR_HITS)) map.addLayer({
        id: LYR_HITS, type: "symbol", source: SRC_HITS,
        layout: { "icon-image": ["concat", "pin-", ["get", "kind"]], "icon-anchor": "bottom", "icon-size": 0.95, "icon-allow-overlap": true, "symbol-sort-key": ["-", 1000, ["get", "i"]],
          "text-field": ["get", "name"], "text-font": font, "text-size": 12.5, "text-anchor": "top", "text-offset": [0, 0.3], "text-max-width": 8, "text-optional": true },
        paint: tpaint,
      });
    }
    addKindImages();
    rankCap = rankCapFor(map.getZoom());
    applyPoi();

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
    if (map.isStyleLoaded() && (!map.getSource(SRC_TRAIL) || !map.getLayer(LYR_TRAIL) || !map.getSource(SRC_TRIPS))) addOverlays();
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
    } catch (e) { toast("PC script not reachable"); }
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
    for (const sw of document.querySelectorAll(".switch")) if (sw.dataset.opt) sw.setAttribute("aria-checked", String(!!S[sw.dataset.opt]));
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
  function openSheet() { closeTrips(); closeNav(); renderStyleList(); renderSwitches(); renderCamSeg(); $("sheet").classList.remove("hidden"); $("scrim").classList.remove("hidden"); }
  function closeSheet() { $("sheet").classList.add("hidden"); $("scrim").classList.add("hidden"); closeTrips(); closeNav(); }

  $("camSeg").onclick = e => { const c = e.target.dataset && e.target.dataset.cam; if (c) setCam(c, true); };
  $("unitSeg").onclick = e => { const u = e.target.dataset && e.target.dataset.unit; if (u) { S.units = u; save(); renderSwitches(); renderHud(); } };
  for (const sw of document.querySelectorAll(".switch[data-opt]")) {
    sw.onclick = () => {
      const k = sw.dataset.opt;
      if (k.startsWith("poi_")) { S[k] = !S[k]; save(); renderSwitches(); applyPoi(); return; } S[k] = !S[k]; save(); renderSwitches();
      if (k === "buildings") { S.buildings = !S.buildings; toggle3d(); return; }
      if (k === "trail") { applyTrail(); trail.dirty = true; }
      if (k === "fog") { fog.dirty = true; map.triggerRepaint(); }
      if (k === "place") showPlace(lastPlace);
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


  // ---------------------------------------------------------------- street and place
  let lastPlace = null, placeKey = "";
  function showPlace(p) {
    const road = (p && p.road) || "";
    const area = p ? [p.area, p.city].filter((x, i, a) => x && a.indexOf(x) === i).join(", ") : "";
    const on = S.place && !!(road || area);
    const key = on ? road + "|" + area : "";
    if (key !== placeKey) {
      placeKey = key;
      if (on) {
        $("placeRoad").textContent = road || area;
        $("placeArea").textContent = road ? area : "";
        $("placeArea").classList.toggle("hidden", !(road && area));
      }
    }
    $("place").classList.toggle("hidden", !on);
    document.body.classList.toggle("plc", on);
  }

  // ---------------------------------------------------------------- safe home
  const home = { data: null, r: 200, dragging: false, key: "" };
  const rFromSlider = v => Math.max(20, Math.round(20 * Math.pow(50, v / 100) / 10) * 10);
  const sliderFromR = r => Math.round(100 * Math.log(r / 20) / Math.log(50));
  const rOut = r => imperial() ? Math.round(r * 3.28084 / 10) * 10 + " ft" : r + " m";
  function homeGeoJSON() {
    const h = home.data;
    if (!h) return { type: "FeatureCollection", features: [] };
    const ring = [], dLat = h.r / 111320, dLon = h.r / (111320 * Math.cos(toRad(h.lat)));
    for (let i = 0; i <= 72; i++) { const a = i / 72 * 2 * Math.PI; ring.push([h.lon + dLon * Math.sin(a), h.lat + dLat * Math.cos(a)]); }
    return { type: "FeatureCollection", features: [{ type: "Feature", properties: {}, geometry: { type: "Polygon", coordinates: [ring] } }] };
  }
  function renderHome() {
    const h = home.data;
    $("homeRv").textContent = rOut(h ? h.r : home.r);
    $("homeSmall").textContent = h ? `Home is set (${rOut(h.r)} radius)` : "No home set";
    $("btnHomeClear").classList.toggle("hidden", !h);
    if (!home.dragging) $("homeR").value = sliderFromR(h ? h.r : home.r);
    const src = !OBS && map.getSource(SRC_HOME); if (src) src.setData(homeGeoJSON());
  }
  function onHome(h) {
    const key = h ? `${h.lat},${h.lon},${h.r}` : "";
    if (key === home.key) return;
    home.key = key; home.data = h || null; if (h) home.r = h.r;
    renderHome();
  }
  async function postHome(body) {
    try {
      const r = await fetch("/home", { method: "POST", body: JSON.stringify(body) });
      if (!r.ok) throw new Error();
      onHome((await r.json()).home);
    } catch (e) { toast("Could not save: PC script not reachable"); }
  }
  function setHomeAt(lat, lon) { postHome({ lat, lon, r: home.r }); toast("Safe home set"); }
  $("homeR").oninput = e => {
    home.dragging = true; home.r = rFromSlider(+e.target.value); $("homeRv").textContent = rOut(home.r);
    if (home.data) { home.data = Object.assign({}, home.data, { r: home.r }); const src = !OBS && map.getSource(SRC_HOME); if (src) src.setData(homeGeoJSON()); }
  };
  $("homeR").onchange = () => { home.dragging = false; if (home.data) postHome({ lat: home.data.lat, lon: home.data.lon, r: home.r }); };
  $("btnHomeHere").onclick = () => {
    const s = st.sample; if (!s) { toast("No position yet. Drive first"); return; }
    setHomeAt(s.lat, s.lng);
  };
  $("btnHomePick").onclick = () => {
    closeSheet(); toast("Tap the map to place your home"); home.picking = true;
    map.once("click", e => { home.picking = false; setHomeAt(e.lngLat.lat, e.lngLat.lng); });
  };
  $("btnHomeClear").onclick = () => { postHome({ clear: true }); toast("Safe home removed"); };
  // ---------------------------------------------------------------- trips
  const trips = { list: [], cur: null, shown: new Map(), timer: 0, armed: null, colorI: 0 };
  const TRIP_COLORS = ["#4ea8ff", "#b58cff", "#3ddc84", "#ffd24a", "#ff6fae", "#2fd4d4"];
  function tripsGeoJSON() {
    const features = [];
    for (const t of trips.shown.values()) features.push({ type: "Feature", properties: { c: t.color }, geometry: { type: "MultiLineString", coordinates: t.segs.filter(x => x.length > 1) } });
    return { type: "FeatureCollection", features };
  }
  function applyTrips() { const src = map.getSource(SRC_TRIPS); if (src) src.setData(tripsGeoJSON()); }
  const fmtWhen = t => new Date(t * 1000).toLocaleString(undefined, { weekday: "short", day: "numeric", month: "short", hour: "2-digit", minute: "2-digit" });
  function tripEl(t) {
    const live = !!t.live, shown = trips.shown.get(t.id);
    const el = document.createElement("div");
    el.className = "tcard" + (live ? " live" : "") + (shown ? " shown" : "");
    const mk = (tag, cls, txt) => { const e = document.createElement(tag); if (cls) e.className = cls; if (txt != null) e.textContent = txt; return e; };
    const head = mk("div", "thead"), dot = mk("span", "tdot");
    if (shown) dot.style.background = shown.color;
    head.append(dot, mk("b", "ttitle", live ? "Current trip" : fmtWhen(t.started)));
    if (live) head.append(mk("span", "tbadge", "LIVE"));
    el.append(head);
    const route = [t.from, t.to].filter(Boolean);
    if (live) el.append(mk("div", "troute", (t.dist_m > 0 ? fmtWhen(t.started) + " · " : "") + (route.length ? route.filter((x, i, a) => a.indexOf(x) === i).join(" → ") : "Starts with your next drive")));
    else if (route.length) el.append(mk("div", "troute", route.filter((x, i, a) => a.indexOf(x) === i).join(" → ")));
    const avg = t.moving_s > 1 ? Math.min(t.dist_m / t.moving_s, t.max_ms || Infinity) : 0;
    const stats = mk("div", "tstats");
    for (const [v, l] of [[distOut(t.dist_m), "Distance"], [fmtTime(t.moving_s), "Drive time"], [`${Math.round(spdOut(avg))} ${spdUnit()}`, "Avg"], [`${Math.round(spdOut(t.max_ms))} ${spdUnit()}`, "Max"]]) {
      const sp = mk("span", null, l); sp.prepend(mk("b", null, v)); stats.append(sp);
    }
    el.append(stats);
    const act = mk("div", "tact");
    const btn = (txt, cls, fn) => { const b = mk("button", "btn " + (cls || ""), txt); b.onclick = fn; act.append(b); return b; };
    if (live) {
      btn("New trip", "", () => resetTrip(null, true));
      const del = btn("Discard", "warn", () => confirmDelete(t.id, del));
    } else {
      btn(shown ? "Hide on map" : "Show on map", shown ? "on" : "", () => toggleTrip(t.id));
      btn("Zoom to", "", () => zoomTrip(t.id));
      const a = mk("a", "btn", "GPX"); a.href = `/trips/${t.id}.gpx`; a.download = ""; act.append(a);
      const del = btn("Delete", "warn", () => confirmDelete(t.id, del));
    }
    el.append(act);
    return el;
  }
  function confirmDelete(id, b) {
    if (trips.armed !== id) {
      trips.armed = id; b.classList.add("armed"); b.textContent = "Tap again";
      setTimeout(() => { if (trips.armed === id) { trips.armed = null; renderTrips(); } }, 3000);
      return;
    }
    trips.armed = null;
    fetch("/trips/delete", { method: "POST", body: JSON.stringify({ id }) })
      .then(() => { trips.shown.delete(id); applyTrips(); loadTrips(); if (trips.cur && trips.cur.id === id) { trail.segs = []; trail.segIds = []; trail.n = 0; trail.dirty = true; } })
      .catch(() => toast("PC script not reachable"));
  }
  function renderTrips() {
    const box = $("tripList"); if (!box || $("tripsSheet").classList.contains("hidden")) return;
    box.replaceChildren();
    if (trips.cur) box.append(tripEl(trips.cur));
    for (const t of trips.list) box.append(tripEl(t));
    if (!trips.list.length) { const e = document.createElement("div"); e.className = "tempty"; e.textContent = "Finished trips appear here. Drive at least 100 m and start a new trip, or restart the script."; box.append(e); }
    $("btnTripsHide").classList.toggle("hidden", !trips.shown.size);
  }
  async function loadTrips() {
    try {
      const r = await fetch("/trips?_=" + Date.now(), { cache: "no-store" });
      const j = await r.json();
      trips.list = j.trips || []; trips.cur = j.current || null;
      for (const id of [...trips.shown.keys()]) if (!trips.list.some(t => t.id === id)) trips.shown.delete(id);
      applyTrips(); renderTrips();
    } catch (e) { toast("PC script not reachable"); }
  }
  async function fetchTripLine(id) {
    let t = trips.shown.get(id);
    if (t) return t;
    const r = await fetch(`/trips/${id}?_=${Date.now()}`, { cache: "no-store" });
    if (!r.ok) throw new Error();
    const j = await r.json(), segs = [], ids = [];
    for (const [la, lo, sg] of j.pts) { if (ids[ids.length - 1] !== sg) { segs.push([]); ids.push(sg); } segs[segs.length - 1].push([lo, la]); }
    t = { segs, color: TRIP_COLORS[trips.colorI++ % TRIP_COLORS.length] };
    return t;
  }
  async function toggleTrip(id) {
    if (trips.shown.has(id)) { trips.shown.delete(id); applyTrips(); renderTrips(); return; }
    try { trips.shown.set(id, await fetchTripLine(id)); applyTrips(); renderTrips(); }
    catch (e) { toast("Could not load that trip"); }
  }
  async function zoomTrip(id) {
    try {
      let t = trips.shown.get(id);
      if (!t) { t = await fetchTripLine(id); trips.shown.set(id, t); applyTrips(); }
      const b = new maplibregl.LngLatBounds();
      for (const seg of t.segs) for (const p of seg) b.extend(p);
      if (b.isEmpty()) return;
      closeTrips(); goFree();
      fitTo(b, 70, 900);
    } catch (e) { toast("Could not load that trip"); }
  }
  function openTrips() {
    $("sheet").classList.add("hidden"); closeNav();
    $("tripsSheet").classList.remove("hidden"); $("scrim").classList.remove("hidden");
    loadTrips(); clearInterval(trips.timer); trips.timer = setInterval(loadTrips, 5000);
  }
  function closeTrips() {
    $("tripsSheet").classList.add("hidden"); clearInterval(trips.timer); trips.armed = null;
    if ($("sheet").classList.contains("hidden") && $("navSheet").classList.contains("hidden")) $("scrim").classList.add("hidden");
  }
  $("btnTrips").onclick = () => $("tripsSheet").classList.contains("hidden") ? openTrips() : closeTrips();
  $("btnTripsClose").onclick = closeTrips;
  $("btnTripNew").onclick = () => resetTrip(null, true);
  $("btnTripsHide").onclick = () => { trips.shown.clear(); applyTrips(); renderTrips(); };


  // ---------------------------------------------------------------- search, routes, navigation
  async function api(path, body, ms) {
    let r; const ac = new AbortController(), tm = setTimeout(() => ac.abort(), ms || 40000);
    try { r = await fetch(path, body ? { method: "POST", body: JSON.stringify(body), signal: ac.signal } : { cache: "no-store", signal: ac.signal }); }
    catch (e) { throw new Error(e.name === "AbortError" ? "The request took too long" : "PC script not reachable"); }
    finally { clearTimeout(tm); }
    let j = {}; try { j = await r.json(); } catch (e) {}
    if (!r.ok) throw new Error(j.error || "Request failed");
    return j;
  }
  const nav = { mode: "idle", view: "search", stops: [], route: null, hits: [], res: null, place: null, busy: false, idx: 0, along: 0, nextStop: 0, off: 0, lastReroute: 0, said: new Set(), markers: [], cat: "", pinTimer: 0 };
  const shortDist = m => imperial() ? (m * 3.28084 < 1000 ? Math.round(m * 3.28084 / 10) * 10 + " ft" : (m / 1609.344).toFixed(m < 16093 ? 1 : 0) + " mi")
    : (m < 1000 ? Math.round(m / 10) * 10 + " m" : (m / 1000).toFixed(m < 10000 ? 1 : 0) + " km");
  const fmtMin = s => { const m = Math.max(1, Math.round(s / 60)); return m < 60 ? m + " min" : Math.floor(m / 60) + " h " + String(m % 60).padStart(2, "0") + " min"; };
  const fmtClock = t => new Date(t).toLocaleTimeString(undefined, { hour: "2-digit", minute: "2-digit" });
  const carPos = () => st.sample ? { lat: st.sample.lat, lon: st.sample.lng } : null;

  // ---- map layers: route, results, markers
  function routeGeoJSON() {
    const r = nav.route;
    return { type: "FeatureCollection", features: r ? [{ type: "Feature", properties: {}, geometry: { type: "LineString", coordinates: r.ll } }] : [] };
  }
  const FUELS = [["diesel", "Diesel"], ["e10", "E10"], ["e5", "E5 / 95"], ["e98", "Super 98"], ["lpg", "LPG"], ["e85", "E85"]];
  const fmtPrice = (v, cur) => { cur = cur || "EUR"; const d = cur === "MXN" ? 2 : 3; try { return v.toLocaleString(undefined, { style: "currency", currency: cur, minimumFractionDigits: d, maximumFractionDigits: d }); } catch (e) { return v.toFixed(d) + " " + cur; } };
  const hitLabel = h => { const p = h.prices && h.prices[S.fuelType]; return p ? (h.name || "") + "\n" + fmtPrice(p, h.cur) : (h.name || ""); };
  function hitsGeoJSON() {
    return { type: "FeatureCollection", features: nav.hits.map((h, i) => ({ type: "Feature", properties: { i, kind: kindOfHit(h), name: hitLabel(h) }, geometry: { type: "Point", coordinates: [h.lon, h.lat] } })) };
  }
  function mansGeoJSON() {   // the turns still ahead, as arrow badges on the route
    const r = nav.route, from = nav.mode === "drive" ? nav.idx : 0;
    return { type: "FeatureCollection", features: !r ? [] : r.mans.filter(m => TURN_BADGE.has(m.type) && m.idx > from && r.ll[m.idx]).map(m => ({ type: "Feature", properties: { k: turnKey(m.type), i: m.idx }, geometry: { type: "Point", coordinates: r.ll[m.idx] } })) };
  }
  function applyMans() { const a = map.getSource(SRC_MANS); if (a) a.setData(mansGeoJSON()); }
  function applyRoute() { const a = map.getSource(SRC_ROUTE); if (a) a.setData(routeGeoJSON()); applyMans(); drawMarkers(); renderScope(); }
  function applyHits() { const a = map.getSource(SRC_HITS); if (a) a.setData(hitsGeoJSON()); }
  function drawMarkers() {
    for (const m of nav.markers) m.remove();
    nav.markers = [];
    const mk = (cls, txt, lat, lon) => { const el = document.createElement("div"); el.className = "mk " + cls; el.textContent = txt; nav.markers.push(new maplibregl.Marker({ element: el }).setLngLat([lon, lat]).addTo(map)); };
    if (nav.mode === "idle" && !nav.route) return;
    nav.stops.forEach((p, i) => { if (i < nav.nextStop && nav.mode === "drive") return; mk(i === nav.stops.length - 1 ? "end" : "", i === nav.stops.length - 1 ? "⚑" : String(i + 1), p.lat, p.lon); });
  }
  const poiLayerIds = () => ((map.getStyle() && map.getStyle().layers) || []).map(l => l.id).filter(id => id.startsWith("bne-poi-"));
  function applyRankCap() {
    const c = rankCapFor(map.getZoom()); if (c === rankCap) return; rankCap = c;
    for (const id of poiLayerIds()) if (POI_BASE[id]) map.setFilter(id, ["all", POI_BASE[id], ["<=", ["coalesce", ["get", "rank"], 99], c]]);
  }
  map.on("zoom", () => { try { applyRankCap(); } catch (e) {} });
  window.addEventListener("resize", () => { try { applyRankCap(); } catch (e) {} });
  function applyPoi() {
    const any = POI_GROUPS.some(g => S["poi_" + g.id]);
    for (const id of poiLayerIds()) { const g = id.split("-")[2]; map.setLayoutProperty(id, "visibility", S["poi_" + g] ? "visible" : "none"); }
    const layers = (map.getStyle() && map.getStyle().layers) || [];
    for (const l of layers) if (l["source-layer"] === "poi" && !l.id.startsWith("bne-")) { try { map.setLayoutProperty(l.id, "visibility", any ? "none" : "visible"); } catch (e) {} }
  }
  function renderPoiRows() {
    const box = $("poiRows"); box.replaceChildren();
    for (const g of POI_GROUPS) {
      const row = document.createElement("div"); row.className = "row";
      const lab = document.createElement("label"); lab.textContent = g.name;
      const ic = document.createElement("span"); ic.style.cssText = "display:inline-block;width:22px;height:22px;vertical-align:middle;margin-right:9px"; ic.innerHTML = kindSvg(g.rules[0][0], false, 22); lab.prepend(ic);
      const sw = document.createElement("button"); sw.className = "switch"; sw.dataset.opt = "poi_" + g.id; sw.setAttribute("role", "switch");
      sw.onclick = () => { S["poi_" + g.id] = !S["poi_" + g.id]; save(); renderSwitches(); applyPoi(); };
      row.append(lab, sw); box.append(row);
    }
  }
  renderPoiRows();
  renderKindImages();

  // ---- the bottom sheet (search / place / plan)
  const sheetEl = $("navSheet"), areaBtn = $("btnArea");
  function openNav(view) {
    $("sheet").classList.add("hidden"); $("tripsSheet").classList.add("hidden"); clearInterval(trips.timer); $("scrim").classList.add("hidden");
    sheetEl.classList.remove("hidden"); document.body.classList.add("navopen");
    setNavView(view || (nav.route && nav.mode === "plan" ? "plan" : "search"));
  }
  function resetSearch() {
    nav.hits = []; nav.res = null; nav.cat = ""; nav.place = null; nav.fuelRes = null; nav.sort = "dist"; applyHits();
    $("navQ").value = ""; $("navResults").replaceChildren(); setChip(""); areaBtn.classList.add("hidden");
  }
  function closeNav() {
    resetSearch();
    sheetEl.classList.add("hidden"); document.body.classList.remove("navopen");
    if ($("sheet").classList.contains("hidden") && $("tripsSheet").classList.contains("hidden")) $("scrim").classList.add("hidden");
  }
  // free map area while the panel is open: phones lose the bottom part, big screens the left side
  function navPad() {
    const r = sheetEl.getBoundingClientRect(), open = !sheetEl.classList.contains("hidden"), wide = innerWidth >= 760;
    const nb = $("navBar"), hud = $("hud");
    const top = nb.classList.contains("hidden") ? 90 : Math.round(nb.getBoundingClientRect().bottom) + 20;
    const hudH = hud && getComputedStyle(hud).display !== "none" ? Math.max(0, innerHeight - hud.getBoundingClientRect().top) + 16 : 24;
    if (open && wide) return { top, left: Math.round(r.right) + 40, right: 90, bottom: hudH };
    if (open) return { top, left: 40, right: 80, bottom: Math.round(r.height) + 30 };
    return { top, left: 40, right: 90, bottom: hudH };
  }
  function navOffset() {   // for a single place: shifts its centre into the free part of the map
    const r = sheetEl.getBoundingClientRect();
    if (sheetEl.classList.contains("hidden")) return [0, 0];
    return innerWidth >= 760 ? [Math.round(r.right / 2), 0] : [0, -Math.round(r.height / 2)];
  }
  const NOPAD = { top: 0, bottom: 0, left: 0, right: 0 };
  function fitTo(b, pad, duration) {
    const old = map.getPadding();
    map.setPadding(NOPAD);   // the chase camera keeps a map padding: measure without it, then animate it away
    let cam = null;
    try { cam = map.cameraForBounds(b, { padding: pad, maxZoom: 16 }); } catch (e) {}
    map.setPadding(old);
    if (cam) map.easeTo({ center: cam.center, zoom: cam.zoom, padding: NOPAD, pitch: 0, bearing: 0, duration });
  }
  function fitRoute() {
    if (!nav.route) return;
    const b = new maplibregl.LngLatBounds();
    for (const p of nav.route.ll) b.extend(p);
    goFree(); fitTo(b, navPad(), 800);
  }
  function setNavView(v) {
    nav.view = v;
    $("navSearchView").classList.toggle("hidden", v !== "search");
    $("navPlaceView").classList.toggle("hidden", v !== "place");
    $("navPlanView").classList.toggle("hidden", v !== "plan");
    if (v === "search") renderScope();
    $("navTitle").textContent = v === "plan" ? "Route" : v === "place" ? "Place" : nav.mode === "drive" ? "Search along the way" : "Search";
    if (v === "plan") { nav.hits = []; applyHits(); areaBtn.classList.add("hidden"); renderPlan(); } else if (v === "place") renderPlace();
  }
  $("btnSearch").onclick = () => sheetEl.classList.contains("hidden") ? openNav() : closeNav();
  $("btnNavClose").onclick = closeNav;
  $("nbSearch").onclick = () => openNav("search");

  const mkEl = (tag, cls, txt) => { const e = document.createElement(tag); if (cls) e.className = cls; if (txt != null) e.textContent = txt; return e; };
  const mkBtn = (txt, cls, fn) => { const b = mkEl("button", "btn " + (cls || ""), txt); b.type = "button"; b.onclick = fn; return b; };

  // ---- search: free text is ranked on the PC script (importance divided by distance), categories come from the map tiles first
  const mapCenterPos = () => ({ lat: map.getCenter().lat, lon: map.getCenter().lng });
  const refPos = () => carPos() || mapCenterPos();
  const UI_LANG = "en-US", langTag = () => encodeURIComponent(UI_LANG);   // English everywhere for now (more languages later)
  const here = ref => `lat=${ref.lat.toFixed(5)}&lon=${ref.lon.toFixed(5)}`;
  function tileSearch(cat, ref) {
    const def = CATS[cat], vs = vectorSourceId();
    if (!def || !vs) return [];
    const conds = def.tile.map(([cls, sub]) => sub ? ["all", ["==", ["get", "class"], cls], ["in", ["get", "subclass"], ["literal", sub]]] : ["==", ["get", "class"], cls]);
    let feats = [];
    try { feats = map.querySourceFeatures(vs, { sourceLayer: "poi", filter: ["any", ...conds] }); } catch (e) { return []; }
    const seen = new Set(), out = [];
    for (const f of feats) {
      if (!f.geometry || f.geometry.type !== "Point") continue;
      const [lon, lat] = f.geometry.coordinates, nm = f.properties.name || "", key = nm + "|" + lat.toFixed(4) + "|" + lon.toFixed(4);
      if (seen.has(key)) continue; seen.add(key);
      const d = dist([ref.lon, ref.lat], [lon, lat]);
      if (d > 15000) continue;
      if (nm && out.some(o => o.name === nm && dist([o.lon, o.lat], [lon, lat]) < 80)) continue;   // the same place cut by a tile border
      out.push({ name: nm || def.n.replace(/s$/, ""), addr: "", lat, lon, type: f.properties.class || "", kind: def.kind, dist_m: Math.round(d) });
    }
    return out;
  }
  const mergeHits = (a, b) => { const out = a.slice(); for (const h of b) if (!out.some(x => dist([x.lon, x.lat], [h.lon, h.lat]) < 60)) out.push(h); return out; };
  function fitHits(ref) {
    if (!nav.hits.length || nav.mode === "drive") return;
    const b = new maplibregl.LngLatBounds([ref.lon, ref.lat], [ref.lon, ref.lat]);
    for (const h of nav.hits.slice(0, 10)) b.extend([h.lon, h.lat]);
    goFree(); fitTo(b, navPad(), 700);
  }
  const routeActive = () => !!nav.route && (nav.mode === "drive" || nav.mode === "plan");
  function renderScope() {
    const on = routeActive(), bar = $("navScope"); if (!bar) return;
    bar.classList.toggle("hidden", !on);
    for (const b of bar.children) b.classList.toggle("on", b.dataset.sc === (nav.scope || "along"));
  }
  $("navScope").onclick = e => { const b = e.target.closest("button"); if (!b) return; nav.scope = b.dataset.sc; renderScope(); if (nav.cat) runCategory(nav.cat); };
  // places beside the route AHEAD of the car (Overpass corridor around the route), nearest first
  async function runAlong(cat) {
    const def = CATS[cat], r = nav.route; nav.cat = cat; setChip(cat); areaBtn.classList.add("hidden");
    const drive = nav.mode === "drive", i0 = drive ? nav.idx : 0, from = drive ? nav.along : 0, AHEAD = 36000, CH = 12000;
    const token = nav.res = { title: def.n + " along the route", cat, along: true, src: "online", loading: true, ahead: Math.min(AHEAD, Math.max(0, r.total - from)) };
    const seen = new Map(), mine = () => nav.res === token;
    const take = list => {
      for (const h of list) {
        const q = project(h.lat, h.lon, i0, r.ll.length - 1), ahead = q.along - from;
        if (q.d > 1500 || ahead < (drive ? 150 : 0) || ahead > AHEAD) continue;
        const key = h.lat.toFixed(4) + "," + h.lon.toFixed(4); if (seen.has(key)) continue;
        seen.set(key, Object.assign(h, { kind: OSM_KIND[h.type] || def.kind, dist_m: Math.round(ahead), off_m: Math.round(q.d), ahead: true, name: h.name || def.n.replace(/s$/, "") }));
      }
      nav.hits = [...seen.values()].sort((a, b) => (a.dist_m + 5 * a.off_m) - (b.dist_m + 5 * b.off_m)).slice(0, 40);
      sortHits(); applyHits(); renderHits();
    };
    take(tileSearch(cat, refPos()));                      // instantly: what the loaded map already knows
    if (!seen.size) renderHits();
    for (let k = Math.floor(from / CH); k * CH < from + AHEAD && k * CH < r.total; k++) {   // then the route ahead, piece by piece (pieces are cached on the PC)
      if (!mine()) return;
      const pts = []; let lastA = -1e9;
      for (let i = Math.max(0, i0 - 1); i < r.ll.length; i++) { const a = r.cum[i]; if (a < k * CH) continue; if (a > (k + 1) * CH + 250) break; if (a - lastA >= 250 || i === r.ll.length - 1) { pts.push([r.ll[i][1], r.ll[i][0]]); lastA = a; } }
      if (pts.length < 2) { if (pts.length === 1 && r.ll.length > 1) pts.unshift([r.ll[Math.max(0, r.ll.length - 2)][1], r.ll[Math.max(0, r.ll.length - 2)][0]]); else continue; }
      try { const j = await api("/along", { points: pts, cat, radius: 1200 }, 40000); if (!mine()) return; take(j.hits || []); }
      catch (e) { if (!mine()) return; token.error = e.message + (seen.size ? ". Showing what was found so far." : "."); break; }
    }
    if (!mine()) return;
    token.loading = false; renderHits();
    if (cat === "fuel") attachPrices(token, refPos(), false);
  }
  async function runCategory(cat, where, online, area) {
    const def = CATS[cat]; if (!def) return;
    if (routeActive() && (nav.scope || "along") === "along" && !where && !area) return runAlong(cat);
    nav.cat = cat; setChip(cat); areaBtn.classList.add("hidden");
    const box = $("navResults"); box.replaceChildren(mkEl("div", "tempty", "Searching…"));
    const ref = area || where || refPos();
    let hits = where && !area ? [] : tileSearch(cat, ref), src = "map";
    if (hits.length < 5 || online) {
      try {
        const on = ((await api(`/search?${here(ref)}&cat=${cat}&lang=${langTag()}`)).hits || []).map(h => Object.assign(h, { kind: OSM_KIND[h.type] || def.kind }));
        hits = mergeHits(hits, on); src = "online";
      } catch (e) { if (!hits.length) { nav.hits = []; applyHits(); box.replaceChildren(mkEl("div", "tempty", e.message)); return; } }
    }
    hits.sort((a, b) => a.dist_m - b.dist_m);
    nav.hits = hits.slice(0, 40);
    nav.res = { title: def.n + (area ? " in this area" : where ? " near " + where.name : " nearby"), cat, where, src, area };
    nav.fuelRes = null; sortHits(); applyHits(); renderHits(); if (!area) fitHits(ref);
    if (cat === "fuel") attachPrices(nav.res, ref, true);
  }
  async function doSearch(q, area) {
    const ref = area || refPos(), box = $("navResults"); areaBtn.classList.add("hidden");
    nav.cat = ""; setChip(""); box.replaceChildren(mkEl("div", "tempty", "Searching…"));
    try { nav.hits = (await api(`/search?${here(ref)}&q=${encodeURIComponent(q)}&lang=${langTag()}`)).hits || []; }
    catch (e) { nav.hits = []; box.replaceChildren(mkEl("div", "tempty", e.message)); applyHits(); return; }
    nav.res = { title: "Results for “" + q + "”", cat: "", q, area };
    applyHits(); renderHits(); if (!area) fitHits(ref);
  }
  async function submitQuery(q) {
    const p = parseQuery(q);
    if (p && !p.where) return runCategory(p.cat);
    if (p && p.where) {
      const box = $("navResults"); box.replaceChildren(mkEl("div", "tempty", "Searching…"));
      try {
        const g = ((await api(`/search?${here(refPos())}&q=${encodeURIComponent(p.where)}&lang=${langTag()}`)).hits || [])[0];
        if (!g) { box.replaceChildren(mkEl("div", "tempty", "Place not found: " + p.where)); return; }
        return runCategory(p.cat, { lat: g.lat, lon: g.lon, name: g.name });
      } catch (e) { box.replaceChildren(mkEl("div", "tempty", e.message)); return; }
    }
    return doSearch(q);
  }
  function hitRow(h, onclick) {
    const k = kindOfHit(h), row = mkEl("button", "hit"); row.type = "button";
    const hi = mkEl("span", "hi"); hi.innerHTML = kindSvg(k, false, 36);   // our own fixed drawing
    const label = k !== "place" ? KIND[k][2] : (h.type && h.type !== "yes" ? h.type.replace(/_/g, " ").replace(/^./, c => c.toUpperCase()) : "");
    const price = h.prices && h.prices[S.fuelType], cheap = price && nav.cheapest === price && nav.res && nav.res.cat === "fuel";
    const t = mkEl("div", "ht"), nm = mkEl("b", null, h.name || label);
    if (cheap) nm.append(mkEl("span", "tag", "Cheapest"));
    t.append(nm, mkEl("span", null, [label, h.open === false ? "Closed" : "", h.off_m != null ? shortDist(h.off_m) + " off route" : "", h.addr].filter(Boolean).join(" · ")));
    row.append(hi, t);
    if (price || h.dist_m != null) {
      const em = mkEl("em", "pr" + (cheap ? " cheap" : ""));
      if (price) em.append(mkEl("b", null, fmtPrice(price, h.cur)));
      if (h.dist_m != null) em.append(price ? mkEl("small", null, (h.ahead ? "in " : "") + shortDist(h.dist_m)) : document.createTextNode((h.ahead ? "in " : "") + shortDist(h.dist_m)));
      row.append(em);
    }
    row.onclick = onclick; return row;
  }
  // fuel prices: the list gets a fuel type choice, "Cheapest" sorting and a short note when prices are not available here
  const PRICE_SRC = { DE: "Tankerkönig (CC BY 4.0)", AT: "E-Control", FR: "data.economie.gouv.fr", ES: "Ministerio para la Transición Ecológica", IT: "MIMIT (daily snapshot of 08:00, not live)", WA: "FuelWatch WA (CC BY 4.0, price of the day)", MX: "CRE México", SE: "fuel-prices.eu", DK: "fuel-prices.eu", GR: "fuel-prices.eu", HR: "fuel-prices.eu", IS: "fuel-prices.eu", SI: "fuel-prices.eu", RO: "fuel-prices.eu" };
  function sortHits() {
    const p = h => h.prices && h.prices[S.fuelType];
    nav.hits.sort(nav.sort === "price" && nav.res && nav.res.cat === "fuel" ? (a, b) => (p(a) ? 0 : 1) - (p(b) ? 0 : 1) || (p(a) || 0) - (p(b) || 0) || a.dist_m - b.dist_m : (a, b) => a.dist_m - b.dist_m);
    const ps = nav.hits.map(p).filter(Boolean); nav.cheapest = ps.length > 1 ? Math.min(...ps) : 0;
  }
  function fuelBar(box) {
    const fr = nav.fuelRes; if (!fr) return;
    if (!fr.supported) { box.append(mkEl("div", "pnote", "No fuel prices for this country. " + (fr.note || ""))); return; }
    const have = new Set(fr.stations.flatMap(s => Object.keys(s.prices)));
    const bar = mkEl("div", "pbar");
    for (const [k, t] of FUELS) if (have.has(k)) { const b = mkEl("button", S.fuelType === k ? "on" : "", t); b.type = "button"; b.onclick = () => { S.fuelType = k; save(); sortHits(); applyHits(); renderHits(); }; bar.append(b); }
    if (have.size) { const b = mkEl("button", nav.sort === "price" ? "on" : "", "Cheapest first"); b.type = "button"; b.onclick = () => { nav.sort = nav.sort === "price" ? "dist" : "price"; sortHits(); renderHits(); }; bar.append(b); }
    if (bar.children.length) box.append(bar);
    if (fr.needs_key.length) box.append(mkEl("div", "pnote", "Prices for Germany need your own free Tankerkönig key on the PC (README, section Fuel prices)."));
    if (fr.errors.length && !have.size) box.append(mkEl("div", "pnote", "Prices: " + fr.errors[0]));
    const src = [...new Set(fr.stations.map(s => s.src))].map(c => PRICE_SRC[c]).filter(Boolean);
    if (src.length) box.append(mkEl("div", "pnote", "Prices: " + src.join(", ")));
  }
  async function attachPrices(token, ref, add) {
    let res; try { res = await api(`/fuel?${here(ref)}&r=12`); } catch (e) { return; }
    if (nav.res !== token) return;
    nav.fuelRes = res;
    for (const st of res.stations) {
      let best = null, bd = 130;
      for (const h of nav.hits) { const d = dist([h.lon, h.lat], [st.lon, st.lat]); if (d < bd) { bd = d; best = h; } }
      if (best) { best.prices = st.prices; best.open = st.open; best.priceSrc = st.src; best.cur = st.cur; if (!best.name || best.name === "Gas station") best.name = st.name || best.name; }
      else if (add) nav.hits.push({ name: st.name || "Gas station", addr: st.addr, lat: st.lat, lon: st.lon, type: "fuel", kind: "fuel", dist_m: Math.round(dist([ref.lon, ref.lat], [st.lon, st.lat])), prices: st.prices, open: st.open, priceSrc: st.src, cur: st.cur });
    }
    sortHits(); nav.hits = nav.hits.slice(0, 40); applyHits(); renderHits();
  }
  function renderHits() {
    const box = $("navResults"), r = nav.res; box.replaceChildren();
    const wider = r && r.cat && !r.along && r.src !== "online" ? mkBtn("Search wider (online)", "", () => runCategory(r.cat, r.where, true, r.area)) : null;
    if (r && r.along) {
      if (nav.hits.length) { const hd = mkEl("div", "rhead"); hd.append(mkEl("b", null, r.title), mkEl("span", null, String(nav.hits.length))); box.append(hd); }
      if (r.cat === "fuel") fuelBar(box);
      nav.hits.forEach(h => box.append(hitRow(h, () => showPlaceView(h))));
      if (r.loading) box.append(mkEl("div", "tempty", nav.hits.length ? "Searching further ahead…" : "Searching along the route…"));
      else if (r.error) { box.append(mkEl("div", "pnote", r.error)); box.append(mkBtn("Try again", "", () => runAlong(r.cat))); }
      else if (!nav.hits.length) box.append(mkEl("div", "tempty", "Nothing beside the route in the next " + Math.round(r.ahead / 1000) + " km."));
      return;
    }
    if (!nav.hits.length) { box.append(mkEl("div", "tempty", r && r.cat ? "Nothing found here. Zoom in on the map or search wider." : "No results")); if (wider) box.append(wider); return; }
    if (r) { const hd = mkEl("div", "rhead"); hd.append(mkEl("b", null, r.title), mkEl("span", null, String(nav.hits.length))); box.append(hd); }
    if (r && r.cat === "fuel") fuelBar(box);
    nav.hits.forEach(h => box.append(hitRow(h, () => showPlaceView(h))));
    if (wider) box.append(wider);
  }
  function renderSuggest(q) {
    const box = $("navResults"); box.replaceChildren();
    for (const m of matchCats(q).slice(0, 3)) {
      const def = CATS[m.cat], row = mkEl("button", "hit"); row.type = "button";
      const hi = mkEl("span", "hi"); hi.innerHTML = kindSvg(def.kind, false, 36);
      const t = mkEl("div", "ht"); t.append(mkEl("b", null, def.n + " nearby"), mkEl("span", null, "Category: found on the map first, no extra request"));
      row.append(hi, t); row.onclick = () => { $("navQ").value = ""; runCategory(m.cat); }; box.append(row);
    }
    const row = mkEl("button", "hit"); row.type = "button";
    const hi = mkEl("span", "hi"); hi.innerHTML = kindSvg("place", false, 36);
    const t = mkEl("div", "ht"); t.append(mkEl("b", null, "Search “" + q + "”"), mkEl("span", null, "Place, address or business, ranked by how well known and how close"));
    row.append(hi, t); row.onclick = () => submitQuery(q); box.append(row);
  }
  $("navForm").onsubmit = e => { e.preventDefault(); const q = $("navQ").value.trim(); if (q.length >= 2) submitQuery(q); };
  $("navQ").addEventListener("input", () => { const q = $("navQ").value.trim(); if (q.length >= 2) renderSuggest(q); else if (nav.hits.length) renderHits(); else $("navResults").replaceChildren(); });
  function setChip(cat) { for (const b of $("navChips").children) b.classList.toggle("on", b.dataset.cat === cat); }
  for (const [c, t] of CHIPS) { const b = mkEl("button"); b.type = "button"; b.dataset.cat = c; b.innerHTML = kindSvg(CATS[c].kind, false, 24); b.append(document.createTextNode(t)); $("navChips").append(b); }
  $("navChips").onclick = e => { const b = e.target.closest("button"), c = b && b.dataset.cat; if (!c) return; if (nav.cat === c) { resetSearch(); return; } $("navQ").value = ""; runCategory(c); };

  // "Search this area": after the map was moved by hand while a search is on screen
  map.on("moveend", e => {
    if (!e.originalEvent || sheetEl.classList.contains("hidden") || nav.view !== "search" || !nav.res) return;
    const p = navPad(), wide = innerWidth >= 760, nb = $("navBar");
    areaBtn.style.left = (wide ? (p.left + innerWidth - p.right) / 2 : innerWidth / 2) + "px";
    areaBtn.style.top = (nb.classList.contains("hidden") ? 58 : Math.round(nb.getBoundingClientRect().bottom) + 8) + "px";
    areaBtn.classList.remove("hidden");
  });
  areaBtn.onclick = () => { const c = mapCenterPos(), r = nav.res; areaBtn.classList.add("hidden"); if (!r) return; if (r.cat) runCategory(r.cat, null, false, c); else if (r.q) doSearch(r.q, c); };
  // on-screen keyboard: lift the panel above it (the browser only shrinks the visual viewport)
  if (window.visualViewport) {
    const vv = window.visualViewport, kb = () => {
      const k = Math.max(0, Math.round(innerHeight - vv.height - vv.offsetTop)), root = document.documentElement.style;
      root.setProperty("--kb", k + "px"); root.setProperty("--vvh", Math.round(vv.height) + "px");
      if (k > 80 && document.activeElement === $("navQ")) $("navQ").scrollIntoView({ block: "nearest" });
    };
    vv.addEventListener("resize", kb); vv.addEventListener("scroll", kb); kb();
  }

  // ---- a place (from a result, a map pin or a point of interest)
  function showPlaceView(h) { nav.place = h; openNav("place"); goFree(); map.easeTo({ center: [h.lon, h.lat], zoom: Math.max(map.getZoom(), 15), duration: 600, offset: navOffset(), padding: NOPAD, pitch: 0, bearing: 0 }); }
  const DAYS = ["Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"];
  function daySet(txt) {
    const out = new Set();
    for (let part of txt.split(",")) {
      part = part.trim(); const m = part.match(/^(Mo|Tu|We|Th|Fr|Sa|Su)(?:-(Mo|Tu|We|Th|Fr|Sa|Su))?$/); if (!m) continue;
      let a = DAYS.indexOf(m[1]); const b = m[2] ? DAYS.indexOf(m[2]) : a; out.add(a); while (a !== b) { a = (a + 1) % 7; out.add(a); }
    }
    return out;
  }
  // opening hours (OpenStreetMap syntax, the common forms only), judged in the time zone of the place's longitude
  function openStatus(oh, lon) {
    if (!oh) return null;
    if (/^\s*24\/7\s*$/.test(oh)) return { open: true, text: "Open 24 hours" };
    const t = new Date(Date.now() + Math.round(lon / 15) * 3600000), day = t.getUTCDay(), mins = t.getUTCHours() * 60 + t.getUTCMinutes();
    let ranges = null;
    for (const rule of oh.split(";")) {
      const r = rule.trim(); if (!r) continue;
      const m = r.match(/^((?:(?:Mo|Tu|We|Th|Fr|Sa|Su|PH|SH)(?:-(?:Mo|Tu|We|Th|Fr|Sa|Su))?\s*,?\s*)*)(.*)$/), dtxt = m[1].trim(), rest = m[2].trim();
      if (dtxt) { const ds = daySet(dtxt); if (!ds.size || !ds.has(day)) continue; }
      if (/^(off|closed)$/i.test(rest)) { ranges = []; continue; }
      if (/^24\/7$/.test(rest)) { ranges = [[0, 1440]]; continue; }
      const rs = [...rest.matchAll(/(\d{1,2}):(\d{2})\s*-\s*(\d{1,2}):(\d{2})/g)];
      if (!rs.length) return null;
      ranges = rs.map(x => { const a = +x[1] * 60 + +x[2]; let b = +x[3] * 60 + +x[4]; if (b <= a) b += 1440; return [a, b]; });
    }
    if (ranges === null) return { open: false, text: "Closed today" };
    const hm = v => String(Math.floor(v / 60) % 24).padStart(2, "0") + ":" + String(v % 60).padStart(2, "0");
    const cur = ranges.find(([a, b]) => mins >= a && mins < b);
    if (cur) return { open: true, text: "Open · until " + hm(cur[1]) };
    const nxt = ranges.filter(([a]) => a > mins).sort((x, y) => x[0] - y[0])[0];
    return { open: false, text: nxt ? "Closed · opens " + hm(nxt[0]) : "Closed" };
  }
  const safeUrl = u => { try { const x = new URL(/^https?:/i.test(u) ? u : "https://" + u); return /^https?:$/.test(x.protocol) ? x.href : ""; } catch (e) { return ""; } };
  function detRow(label, node) { const r = mkEl("div", "pd-row"), v = mkEl("span"); v.append(node); r.append(mkEl("span", null, label), v); return r; }
  function renderDetails(h) {
    const box = $("placeDet"); if (!box || nav.place !== h) return; box.replaceChildren();
    const d = h.det || {}, t = d.tags || {};
    if (d.photo) { const im = mkEl("img", "ph"); im.src = d.photo; im.alt = h.name || ""; im.loading = "lazy"; im.referrerPolicy = "no-referrer"; im.onerror = () => im.remove(); box.append(im); }
    if (d.about) { const a = mkEl("div", "pd-about", d.about); if (d.wiki && safeUrl(d.wiki)) { const l = mkEl("a", null, " Wikipedia"); l.href = safeUrl(d.wiki); l.target = "_blank"; l.rel = "noopener noreferrer"; a.append(l); } box.append(a); }
    if (h.prices && Object.keys(h.prices).length) {
      for (const [k, n] of FUELS) if (h.prices[k]) { const r = mkEl("div", "pd-price"); r.append(mkEl("span", null, n), mkEl("b", null, fmtPrice(h.prices[k], h.cur))); box.append(r); }
      box.append(mkEl("div", "pnote", "Price from " + (PRICE_SRC[h.priceSrc] || "the national price service") + (h.priceSrc === "IT" || h.priceSrc === "WA" ? "" : ", live")));
    }
    const os = openStatus(t.opening_hours, h.lon);
    if (t.opening_hours) {
      const v = mkEl("div"); if (os) v.append(mkEl("div", os.open ? "pd-open" : "pd-closed", os.text));
      v.append(mkEl("div", null, t.opening_hours.replace(/;\s*/g, "\n").split("\n").join(" · ")));
      box.append(detRow("Hours", v));
    }
    const ph = (t.phone || t["contact:phone"] || "").split(";")[0].replace(/[^0-9+ ()-]/g, "");
    if (ph) { const a = mkEl("a", null, ph); a.href = "tel:" + ph.replace(/[^0-9+]/g, ""); box.append(detRow("Phone", a)); }
    const web = safeUrl(t.website || t["contact:website"] || "");
    if (web) { const a = mkEl("a", null, new URL(web).host.replace(/^www\./, "")); a.href = web; a.target = "_blank"; a.rel = "noopener noreferrer"; box.append(detRow("Website", a)); }
    const more = [t.brand && t.brand !== h.name ? t.brand : "", t.cuisine ? t.cuisine.replace(/[;_]/g, ", ") : "", t.wheelchair === "yes" ? "Wheelchair accessible" : "", t.drive_through === "yes" ? "Drive-through" : "", t.fuels ? "Fuels: " + t.fuels.replace(/_/g, " ").replace(/,/g, ", ") : ""].filter(Boolean);
    if (more.length) box.append(detRow("Details", document.createTextNode(more.join(" · "))));
    if (h.det && !box.children.length) box.append(mkEl("div", "pnote", h.det.found === false ? "OpenStreetMap has no further details for this place." : "No further details for this place."));
    if (!h.det && !h.detErr) box.append(mkEl("div", "pnote", "Loading details…"));
    if (!h.det && h.detErr) { box.append(mkEl("div", "pnote", "Details could not be loaded: " + h.detErr + ".")); box.append(mkBtn("Try again", "", () => { h.detErr = ""; renderDetails(h); loadDetails(h); })); }
  }
  async function loadPrice(h) {   // a station opened from the map or the route list: look up its price by position
    if (h.prices || h.priceTried || (h.kind !== "fuel" && h.type !== "fuel")) return;
    h.priceTried = true;
    try {
      const res = await api(`/fuel?lat=${h.lat.toFixed(5)}&lon=${h.lon.toFixed(5)}&r=3`, null, 25000); let best = null, bd = 130;
      for (const st of res.stations || []) { const d = dist([h.lon, h.lat], [st.lon, st.lat]); if (d < bd) { bd = d; best = st; } }
      if (best) { h.prices = best.prices; h.open = best.open; h.priceSrc = best.src; h.cur = best.cur; }
    } catch (e) { /* no price: the card simply has none */ }
    renderDetails(h);
  }
  async function loadDetails(h) {
    loadPrice(h);
    if (h.det === undefined && !h.detErr) {
      if (h.tags && !(h.tags.wikipedia || h.tags.wikimedia_commons || h.tags.image)) h.det = { tags: h.tags };
      else {
        try { h.det = await api(`/place?lat=${h.lat}&lon=${h.lon}&name=${encodeURIComponent(h.name || "")}`, null, 28000); if (h.tags) h.det.tags = Object.assign({}, h.det.tags, h.tags); }
        catch (e) { h.det = undefined; h.detErr = e.message; }
      }
    }
    renderDetails(h);
  }
  function renderPlace() {
    const h = nav.place, box = $("navPlaceView"); box.replaceChildren();
    if (!h) return;
    box.append(mkEl("div", "nvsum", h.name), mkEl("div", "nvhint", [h.addr, h.dist_m != null ? shortDist(h.dist_m) + (h.ahead ? " ahead" : " away") : ""].filter(Boolean).join(" · ")));
    const row = mkEl("div", "nvbtns");
    if (nav.stops.length && (nav.mode === "plan" || nav.mode === "drive")) {
      row.append(mkBtn("Add as stop", "go nvbtn", () => { addStop(h); }), mkBtn("Make destination", "", () => { nav.stops = [h]; nav.nextStop = 0; if (nav.mode === "drive") closeNav(); replan(nav.mode !== "drive"); }));
    } else row.append(mkBtn("Route here", "go nvbtn", () => { nav.stops = [h]; nav.nextStop = 0; nav.mode = "plan"; replan(true); }));
    row.append(mkBtn("Back", "", () => setNavView(nav.stops.length && nav.route ? "plan" : "search")));
    box.append(row);
    const det = mkEl("div"); det.id = "placeDet"; box.append(det);
    renderDetails(h); loadDetails(h);
  }
  map.on("click", e => {
    if (fog.edit || home.picking) return;
    const ids = [LYR_HITS, ...poiLayerIds()].filter(id => map.getLayer(id)), r = 14;
    const f = ids.length ? map.queryRenderedFeatures([[e.point.x - r, e.point.y - r], [e.point.x + r, e.point.y + r]], { layers: ids })[0] : null;
    if (!f) return;
    if (f.layer.id === LYR_HITS) { showPlaceView(nav.hits[f.properties.i]); return; }
    const [lon, lat] = f.geometry.coordinates, c = carPos(), kind = kindOfTile(f.properties.class, f.properties.subclass);
    showPlaceView({ name: f.properties.name || KIND[kind][2], addr: KIND[kind][2], kind, lat, lon, dist_m: c ? Math.round(dist([c.lon, c.lat], [lon, lat])) : null });
  });
  // long press = drop a pin
  const pinCancel = () => clearTimeout(nav.pinTimer);
  map.on("mousedown", e => { pinCancel(); nav.pinTimer = setTimeout(() => dropPin(e.lngLat), 650); });
  map.on("touchstart", e => { pinCancel(); if (e.points && e.points.length > 1) return; nav.pinTimer = setTimeout(() => dropPin(e.lngLat), 650); });
  for (const ev of ["mouseup", "touchend", "touchcancel", "mousemove", "touchmove", "dragstart", "zoomstart", "rotatestart", "pitchstart"]) map.on(ev, pinCancel);
  function dropPin(ll) {
    if (fog.edit || home.picking) return;
    const c = carPos();
    showPlaceView({ name: "Dropped pin", addr: `${ll.lat.toFixed(5)}, ${ll.lng.toFixed(5)}`, lat: ll.lat, lon: ll.lng, dist_m: c ? Math.round(dist([c.lon, c.lat], [ll.lng, ll.lat])) : null });
  }

  // ---- planning
  function prepareRoute(r) {
    const ll = r.shape.map(p => [p[1], p[0]]), cum = [0];
    for (let i = 1; i < ll.length; i++) cum.push(cum[i - 1] + dist(ll[i - 1], ll[i]));
    return { ll, cum, total: cum[cum.length - 1] || r.dist_m, dist_m: r.dist_m, time_s: r.time_s, mans: r.maneuvers, ends: r.ends };
  }
  function insertCost(a, b, p) { return dist([a.lon, a.lat], [p.lon, p.lat]) + dist([p.lon, p.lat], [b.lon, b.lat]) - dist([a.lon, a.lat], [b.lon, b.lat]); }
  function addStop(h) {
    if (nav.stops.length >= 10) { toast("Ten stops are the maximum"); return; }
    // the new stop goes where it adds the least detour (before the destination, never after it)
    const pts = [carPos() || { lat: h.lat, lon: h.lon }, ...nav.stops.slice(nav.nextStop)];
    let best = 0, bestCost = Infinity;
    for (let i = 0; i < pts.length - 1; i++) { const c = insertCost(pts[i], pts[i + 1], h); if (c < bestCost) { bestCost = c; best = i; } }
    nav.stops.splice(nav.nextStop + best, 0, h);
    toast("Stop added: " + h.name);
    if (nav.mode === "drive") closeNav();
    replan(nav.mode !== "drive");
  }
  function firstPt(c) {   // the car, with the direction it is facing when it moves: the route then starts that way
    const s = st.sample, p = { lat: c.lat, lon: c.lon };
    if (s && s.speed > 2.5 && isFinite(s.bearing)) p.heading = Math.round(s.bearing);
    return p;
  }
  async function replan(show) {
    const c = carPos(); if (!c) { toast("No position yet. Drive first"); return; }
    if (nav.busy) return;
    const rest = nav.stops.slice(nav.nextStop);
    if (!rest.length) return;
    nav.busy = true; if (show && !sheetEl.classList.contains("hidden")) setNavView("plan");
    try {
      const r = await api("/route", { points: [firstPt(c), ...rest.map(p => ({ lat: p.lat, lon: p.lon, name: p.name }))], avoid: { highways: S.avoidHw, tolls: S.avoidToll, ferries: S.avoidFerry }, lang: UI_LANG });
      nav.stops = rest; nav.nextStop = 0;
      nav.route = prepareRoute(r); nav.idx = 0; nav.along = 0; nav.said.clear(); nav.off = 0;
      if (nav.mode === "idle") nav.mode = "plan";
      applyRoute(); saveNav();
      if (!sheetEl.classList.contains("hidden") && nav.mode === "plan") setNavView("plan");
      if (show) fitRoute();
      if (nav.mode === "drive") renderNavBar();
    } catch (e) { toast(e.message); if (!nav.route) { nav.stops = []; nav.mode = "idle"; if (!sheetEl.classList.contains("hidden")) setNavView("search"); } else if (!sheetEl.classList.contains("hidden") && nav.mode === "plan") setNavView("plan"); }
    finally { nav.busy = false; }
  }
  function renderPlan() {
    const box = $("navPlanView"); box.replaceChildren();
    const r = nav.route;
    if (nav.busy && !r) { box.append(mkEl("div", "tempty", "Calculating the route…")); return; }
    if (r) { const sm = mkEl("div", "nvsum", `${fmtMin(r.time_s)}`); sm.append(mkEl("small", null, `${distOut(r.dist_m)} · arrive ${fmtClock(Date.now() + r.time_s * 1000)}`)); box.append(sm); }
    nav.stops.forEach((p, i) => {
      const last = i === nav.stops.length - 1, row = mkEl("div", "stoprow");
      row.append(mkEl("div", "sn" + (last ? " end" : ""), last ? "⚑" : String(i + 1)), mkEl("div", "st", p.name));
      if (i > 0) { const up = mkEl("button", null, "↑"); up.type = "button"; up.title = "Earlier"; up.onclick = () => { [nav.stops[i - 1], nav.stops[i]] = [nav.stops[i], nav.stops[i - 1]]; replan(false); }; row.append(up); }
      const x = mkEl("button", null, "✕"); x.type = "button"; x.title = "Remove"; x.onclick = () => { nav.stops.splice(i, 1); if (!nav.stops.length) cancelPlan(); else replan(false); }; row.append(x);
      box.append(row);
    });
    const opts = mkEl("div", "optrow");
    for (const [k, t] of [["avoidHw", "Avoid motorways"], ["avoidToll", "Avoid tolls"], ["avoidFerry", "Avoid ferries"]]) {
      const b = mkEl("button", S[k] ? "on" : "", t); b.type = "button"; b.onclick = () => { S[k] = !S[k]; save(); replan(false); }; opts.append(b);
    }
    box.append(opts);
    const row = mkEl("div", "nvbtns");
    row.append(mkBtn(nav.mode === "drive" ? "Continue" : "Start", "go nvbtn", () => nav.mode === "drive" ? closeNav() : startNav()), mkBtn("Whole route", "", fitRoute), mkBtn("Add stop", "", () => { setChip(""); nav.hits = []; nav.res = null; applyHits(); $("navResults").replaceChildren(); setNavView("search"); $("navQ").focus(); }), mkBtn(nav.mode === "drive" ? "End route" : "Cancel", "warn", () => nav.mode === "drive" ? endNav() : cancelPlan()));
    box.append(row);
    box.append(mkEl("div", "nvhint", "Search a gas station or anything else and tap Add as stop: it is put where it costs the least detour."));
  }
  function cancelPlan() { nav.stops = []; nav.route = null; nav.mode = "idle"; nav.nextStop = 0; applyRoute(); saveNav(); closeNav(); }

  // ---- driving
  function startNav() {
    if (!nav.route) return;
    nav.mode = "drive"; nav.idx = 0; nav.along = 0; nav.nextStop = 0; nav.said.clear(); nav.off = 0;
    document.body.classList.add("navon"); closeNav(); nav.hits = []; applyHits(); drawMarkers();
    recenter(); renderNavBar(); saveNav();
  }
  function endNav(msg) {
    nav.mode = "idle"; nav.route = null; nav.stops = []; nav.nextStop = 0;
    document.body.classList.remove("navon"); $("navBar").classList.add("hidden");
    applyRoute(); saveNav(); closeNav(); if (msg) toast(msg);
  }
  $("nbEnd").onclick = () => endNav("Navigation ended");
  $("nbOverview").onclick = () => { if (nav.route) { closeNav(); fitRoute(); } };
  const ICONS = { 1: "↑", 2: "↱", 3: "↰", 4: "⚑", 5: "⚑", 6: "⚑", 7: "↑", 8: "↑", 9: "↗", 10: "→", 11: "↘", 12: "↷", 13: "↶", 14: "↙", 15: "←", 16: "↖", 17: "↑", 18: "↗", 19: "↖", 20: "↗", 21: "↖", 22: "↑", 23: "↗", 24: "↖", 25: "⤮", 26: "⟳", 27: "⟳", 28: "⛴", 29: "⛴" };
  function project(lat, lng, from, to) {
    // nearest point on the route between segment indexes [from, to): { i, d (m), along (m) }
    const ll = nav.route.ll, cum = nav.route.cum, k = Math.cos(toRad(lat)) * 111320;
    let best = { i: from, d: Infinity, along: 0 };
    for (let i = from; i < Math.min(to, ll.length - 1); i++) {
      const ax = (ll[i][0] - lng) * k, ay = (ll[i][1] - lat) * 110540, bx = (ll[i + 1][0] - lng) * k, by = (ll[i + 1][1] - lat) * 110540;
      const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
      const t = L2 > 0 ? clamp(-(ax * dx + ay * dy) / L2, 0, 1) : 0;
      const d = Math.hypot(ax + t * dx, ay + t * dy);
      if (d < best.d) best = { i, d, along: cum[i] + t * (cum[i + 1] - cum[i]) };
    }
    return best;
  }
  function navTick(lat, lng, speed) {
    const r = nav.route; if (!r || nav.busy) return;
    let q = project(lat, lng, nav.idx, nav.idx + 300);
    if (q.d > 80) { const g = project(lat, lng, 0, r.ll.length - 1); if (g.d < q.d) q = g; }
    if (q.d < 80) { nav.idx = q.i; nav.along = q.along; }
    const now = performance.now();
    if (q.d > 60) {
      if (!nav.off) nav.off = now;
      else if (now - nav.off > 4000 && now - nav.lastReroute > 10000) { nav.lastReroute = now; nav.off = 0; toast("Rerouting…"); replan(false); return; }
    } else nav.off = 0;
    // stops and arrival
    const stop = nav.stops[nav.nextStop];
    if (stop && dist([lng, lat], [stop.lon, stop.lat]) < 40 && nav.idx >= (r.ends[0] || 0) - 6) {
      if (nav.nextStop >= nav.stops.length - 1) { endNav("You have arrived"); return; }
      toast("Stop reached: " + stop.name);
      nav.stops.splice(nav.nextStop, 1); replan(false); return;
    }
    renderNavBar(speed);
  }
  function renderNavBar(speed) {
    const r = nav.route, bar = $("navBar");
    if (!r || nav.mode !== "drive") { bar.classList.add("hidden"); return; }
    bar.classList.remove("hidden"); document.body.classList.add("navon");
    let mi = r.mans.findIndex(m => m.idx > nav.idx);
    if (mi < 0) mi = r.mans.length - 1;
    const m = r.mans[mi] || { text: "", type: 8, idx: 0 };
    const toTurn = Math.max(0, (r.cum[Math.min(m.idx, r.cum.length - 1)] || 0) - nav.along);
    $("nbIcon").innerHTML = turnSvg(m.type, 40, false);
    if (nav.mansAt !== mi) { nav.mansAt = mi; applyMans(); }
    $("nbDist").textContent = shortDist(toTurn);
    $("nbText").textContent = m.text;
    const left = Math.max(0, r.total - nav.along), secs = r.time_s * (left / (r.total || 1));
    $("nbSum").textContent = `${distOut(left)} · ${fmtMin(secs)} · ${fmtClock(Date.now() + secs * 1000)}` + (nav.stops.length > 1 ? ` · ${nav.stops.length} stops` : "");
  }

  // keep a running navigation across a page reload
  function saveNav() { try { sessionStorage.setItem("bne-nav", JSON.stringify(nav.mode === "idle" ? null : { mode: nav.mode, stops: nav.stops.slice(nav.nextStop) })); } catch (e) {} }
  (function resume() {
    let saved = null; try { saved = JSON.parse(sessionStorage.getItem("bne-nav") || "null"); } catch (e) {}
    if (!saved || !saved.stops || !saved.stops.length || OBS) return;
    const t = setInterval(async () => {
      if (!carPos()) return; clearInterval(t);
      nav.stops = saved.stops; nav.nextStop = 0; nav.mode = saved.mode === "drive" ? "drive" : "plan";
      await replan(saved.mode !== "drive");
      if (nav.route && saved.mode === "drive") { document.body.classList.add("navon"); renderNavBar(); toast("Navigation resumed"); }
    }, 700);
  })();

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
    else if (k === "t") $("btnTrips").click();
    else if (k === "/" && !/INPUT|TEXTAREA/.test(document.activeElement.tagName)) { e.preventDefault(); openNav(); }
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
    global rec, fog, geo, nav_enabled, ROUTER_URL
    load_home()
    load_settings()
    load_keys()
    nav_enabled = "--no-nav" not in sys.argv
    if "--router" in sys.argv:
        try:
            ROUTER_URL = sys.argv[sys.argv.index("--router") + 1]
        except IndexError:
            pass
    rec = Recorder()
    fog = Fog()
    if "--no-places" not in sys.argv:
        geo = Places()
        geo.start()
    atexit.register(lambda: (rec.save(force=True), fog.save(force=True), search_cache.save(), save_settings()))
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
    feed = "--feed" in sys.argv or bool(os.environ.get("BNE_FEED"))
    print(f"  Position from:      BeamNG.Earth's feed {FEED_URL}" if feed else
          f"  Relay:              game -> :{LISTEN_PORT} -> server :{SERVER_PORT}")
    print(line)
    t = rec.trip
    print(f"  Total driven:       {rec.odo_m / 1000:.2f} km")
    print(f"  Trips:              {len(rec.trips)} saved (open the list with the trips button on the map)")
    print(f"  Street names:       " + ("off (--no-places)" if not geo else "on (OpenStreetMap Nominatim; --no-places turns it off)"))
    print("  Fuel prices:        " + ("Germany key found; " if fuel_keys.get("tankerkoenig") else "Germany needs a free Tankerkoenig key (README); ") + "most other supported countries need nothing (README)")
    print("  Search + routes:    " + ("off (--no-nav)" if not nav_enabled else f"on (OpenStreetMap Nominatim + Valhalla {ROUTER_URL})"))
    fs = fog.summary()
    print(f"  Explored (fog):     {fs['km2']:.2f} km2, {fs['cells']} cells (edit or reset it on the map: Settings > Fog)")
    print(f"  Data folder:        {DATA_DIR}")
    print(line)
    if feed:
        serve_feed()
    else:
        serve_relay()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Stopped.")
