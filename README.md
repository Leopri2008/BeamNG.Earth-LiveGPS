# BeamNG.Earth Live GPS

Drive in [BeamNG.Earth](https://github.com/TecnaGamer/BeamNG.Earth), and your **Android phone's GPS follows your car in real time**,
as if you were really driving there. Open Google Maps, Waze or any navigation app on your phone and watch it track
your in-game drive on the real-world map.



**No changes to BeamNG.Earth** (neither the mod nor the server EXE). It only listens in on the position updates
the mod already sends to its server.

## How it works

```
BeamNG.drive (BeamNG.Earth mod)
        │  TCP 127.0.0.1:47614   (game started with -bngeport 47614)
        ▼
pc/bne_live_coords.py  ──►  BeamNG.Earth server EXE (127.0.0.1:47613, unchanged)
        │  HTTP :47700 (your Wi-Fi)
        ▼
Android app "BNE Mock GPS"  ──►  Android mock location (GPS / network / fused / Google Play services)
        ▼                                         Live map in any browser: http://PC-IP:47700/map
Google Maps, Waze, …
```

The BeamNG.Earth mod sends a `pose` message with the car's real latitude/longitude about twice per second.
`bne_live_coords.py` sits between game and server as a transparent relay, reads those messages and serves the
latest position as JSON. The app polls it 4×/s and feeds the phone's location 5×/s, extrapolating with speed and
heading in between so the map moves smoothly.

## Requirements

- BeamNG.drive with BeamNG.Earth installed and working
- Python 3.8+ on the PC
- An Android phone (8.0+) on the **same Wi-Fi** as the PC
- iPhone can't use the GPS mock (iOS does not allow mock locations), but the live map works

## Setup

### 1. PC

1. Start the BeamNG.Earth server as usual.
2. Run `python pc/bne_live_coords.py`
   It prints the address for the phone, e.g. `Phone app address:  192.168.178.20:47700`.
   and the live map address, e.g. `Live map (browser): http://192.168.178.20:47700/map`.
   If Windows Firewall asks, allow access on **private networks**.
4. Steam → BeamNG.drive → Properties → Launch options: `-bngeport 47614`
5. Start BeamNG.drive and load BeamNG.Earth. The console shows `Game connected.` and then a `LIVE` line with your coordinates.

To go back to normal, just remove the launch option.

You can also open `http://127.0.0.1:47700/` in a browser, or read `live_position.json`, for your own overlays/tools.

### 2. Phone

1. Download `BNE-Mock-GPS.apk` from the [Releases](../../releases) page and install it. 
2. Enable **Developer options** (Settings → About phone → tap *Build number* 7×).
3. Developer options → **Select mock location app** → **BNE Mock GPS**
   (the app has a button that opens developer options).
4. Open the app, enter the PC address, tap **START**, allow location and notifications.
   The status turns green **LIVE** and shows your coordinates, speed and heading.
5. Open your navigation app and drive.

### 3. Live map (any device, including iPhone)

Open the `Live map` address from the console in any browser on your Wi-Fi, e.g. `http://192.168.178.20:47700/map`
(on the PC itself: `http://127.0.0.1:47700/map`). The Android app also has an **Open live map** button.

- **Map styles:** Dark, Liberty, Bright, Positron, Fiord (free [OpenFreeMap](https://openfreemap.org) tiles, no account)
- **Camera:** North up, Heading up, or **Chase 3D**, drag the map to look around, tap **Recenter** to follow again
- **3D buildings**, auto zoom by speed, your driven route, live speed (km/h or mph) and compass
- **Trip stats:** distance, drive time, average and top speed
- Settings are remembered per browser. Desktop keys: `C` camera, `B` 3D, `S` style, `R` recenter, `F` fullscreen, `U` units

**iPhone / iPad:** for full screen, tap Share → **Add to Home Screen** and open it from there.

**Screen stays on:** on Android, open the map with the **Open live map** button in the app, which keeps the screen on reliably.
In a browser the page tries to keep the screen on too (tap the map once after opening it), but over the Wi-Fi address
browsers don't guarantee it. If it still turns off (e.g. iPhone), set Auto-Lock to "Never" while driving.

**OBS overlay:** add a Browser Source with `http://127.0.0.1:47700/map?obs=1` (map + speedometer only).
Options: `&stats=1` shows trip stats, `&style=liberty`, `&cam=heading`, `&units=imperial`, `&buildings=0`, `&trail=0`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Console never says `Game connected.` | Launch option `-bngeport 47614` missing or game not restarted |
| `BeamNG.Earth server not reachable` | Start the BeamNG.Earth EXE **before** the script |
| `Game connected.` but no coordinates | Load into the world and drive; poses are only sent once the world around you is loaded |
| App shows ERROR `PC not reachable` | Same Wi-Fi? Correct IP? Firewall allowed on private networks? Test `http://PC-IP:47700/` in the phone's browser |
| App shows ERROR `not selected as mock location app` | Developer options → Select mock location app → BNE Mock GPS |
| Google Maps jumps around or shows wrong speed while navigating | Update to v1.1. If it still happens: Settings → Location → turn off *Google Location Accuracy* and Wi-Fi/Bluetooth scanning |
| Live map stays grey or says "Map could not load" | The device needs internet for the map tiles; reload the page |
| Live map says `PC script not reachable` | Script still running? Same Wi-Fi? Firewall allowed on private networks? |

## Notes

- Some apps and games detect mock locations. Games like Pokémon GO ban accounts for it, so don't use it there.
- The HTTP endpoint is reachable by anyone on your local network (read-only, position only).
- Map data © [OpenFreeMap](https://openfreemap.org), © [OpenMapTiles](https://www.openmaptiles.org/), data from [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors.
  
## Credits

- [BeamNG.Earth](https://github.com/TecnaGamer/BeamNG.Earth) by TecnaGamer. This is an unofficial companion tool,
  not affiliated with BeamNG.Earth or BeamNG GmbH.

## License

**All rights reserved.** Free for personal use; credit required when shown publicly.
Re-uploading, redistributing modified or unmodified copies, and commercial use are **not allowed** without
written permission. Please share a link to this repository instead. Full terms: [LICENSE](LICENSE).

If you show it in a video or stream, please credit:
`BeamNG.Earth Live GPS by Leopri2009 – https://github.com/Leopri2008/BeamNG.Earth-LiveGPS`

---

### Kurzanleitung (Deutsch)

1. BeamNG.Earth-Server starten, dann `python pc/bne_live_coords.py`.
2. In Steam bei BeamNG.drive die Startoption `-bngeport 47614` setzen und das Spiel starten.
3. APK aus den Releases installieren, in den Entwickleroptionen als „App für Pseudo-Standorte" wählen.
4. In der App die PC-Adresse aus der Konsole eintragen und auf START tippen. Handy und PC müssen im selben WLAN sein.
5. Live-Karte (auch iPhone): die `Live map`-Adresse aus der Konsole im Browser öffnen, z. B. `http://192.168.178.20:47700/map`.

Lizenz: Alle Rechte vorbehalten. Private Nutzung ist frei, bei öffentlicher Vorstellung sind Credits Pflicht.
Weiterverbreitung und kommerzielle Nutzung nur mit schriftlicher Erlaubnis. Details siehe [LICENSE](LICENSE).

