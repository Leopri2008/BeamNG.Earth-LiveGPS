# BeamNG.Earth Live GPS

Drive in [BeamNG.Earth](https://github.com/TecnaGamer/BeamNG.Earth), and your **Android phone's GPS follows your car in real time**,
as if you were really driving there. Open Google Maps, Waze or any navigation app on your phone and watch it track
your in-game drive on the real-world map.

<!-- Screenshot: put your image at docs/screenshot.png and remove the comment markers
![BeamNG.Earth Live GPS](docs/screenshot.png)
-->

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
Android app "BNE Mock GPS"  ──►  Android mock location (GPS / network / fused)
        ▼
Google Maps, Waze, …
```

The BeamNG.Earth mod sends a `pose` message with the car's real latitude/longitude about twice per second.
`bne_live_coords.py` sits between game and server as a transparent relay, reads those messages and serves the
latest position as JSON. The app polls it 4×/s and feeds the phone's location 5×/s, extrapolating with speed and
heading in between so the map moves smoothly.

## Requirements

- BeamNG.drive with BeamNG.Earth installed and working
- Python 3.8+ on the PC (no extra packages)
- An Android phone (8.0+) on the **same Wi-Fi** as the PC
- iPhone is not supported (iOS does not allow mock locations)

## Setup

### 1. PC

1. Start the BeamNG.Earth server as usual.
2. Run `python pc/bne_live_coords.py`
   It prints the address for the phone, e.g. `Phone app address:  192.168.178.20:47700`.
   If Windows Firewall asks, allow access on **private networks**.
3. Steam → BeamNG.drive → Properties → Launch options: `-bngeport 47614`
4. Start BeamNG.drive and load BeamNG.Earth. The console shows `Game connected.` and then a `LIVE` line with your coordinates.

To go back to normal, just remove the launch option.

You can also open `http://127.0.0.1:47700/` in a browser, or read `live_position.json`, for your own overlays/tools.

### 2. Phone

1. Download `BNE-Mock-GPS.apk` from the [Releases](../../releases) page (or from the latest
   [Actions](../../actions) run) and install it. Or build it yourself: open `android/` in Android Studio and press ▶.
2. Enable **Developer options** (Settings → About phone → tap *Build number* 7×).
3. Developer options → **Select mock location app** → **BNE Mock GPS**
   (the app has a button that opens developer options).
4. Open the app, enter the PC address, tap **START**, allow location and notifications.
   The status turns green **LIVE** and shows your coordinates, speed and heading.
5. Open your navigation app and drive.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Console never says `Game connected.` | Launch option `-bngeport 47614` missing or game not restarted |
| `BeamNG.Earth server not reachable` | Start the BeamNG.Earth EXE **before** the script |
| `Game connected.` but no coordinates | Load into the world and drive; poses are only sent once the world around you is loaded |
| App shows ERROR `PC not reachable` | Same Wi-Fi? Correct IP? Firewall allowed on private networks? Test `http://PC-IP:47700/` in the phone's browser |
| App shows ERROR `not selected as mock location app` | Developer options → Select mock location app → BNE Mock GPS |

## Notes

- Some apps and games detect mock locations. Games like Pokémon GO ban accounts for it, so don't use it there.
- The HTTP endpoint is reachable by anyone on your local network (read-only, position only).
- The release APK is a debug build (signed with a debug key), which is fine for sideloading.

## Credits

- [BeamNG.Earth](https://github.com/TecnaGamer/BeamNG.Earth) by TecnaGamer. This is an unofficial companion tool,
  not affiliated with BeamNG.Earth or BeamNG GmbH.

## License

**All rights reserved.** Free for personal use; credit required when shown publicly.
Re-uploading, redistributing modified or unmodified copies, and commercial use are **not allowed** without
written permission. Please share a link to this repository instead. Full terms: [LICENSE](LICENSE).

If you show it in a video or stream, please credit:
`BeamNG.Earth Live GPS by [YOUR NAME / GITHUB USERNAME] – https://github.com/[YOUR GITHUB USERNAME]/BeamNG.Earth-LiveGPS`

---

### Kurzanleitung (Deutsch)

1. BeamNG.Earth-Server starten, dann `python pc/bne_live_coords.py`.
2. In Steam bei BeamNG.drive die Startoption `-bngeport 47614` setzen und das Spiel starten.
3. APK aus den Releases installieren, in den Entwickleroptionen als „App für Pseudo-Standorte" wählen.
4. In der App die PC-Adresse aus der Konsole eintragen und auf START tippen. Handy und PC müssen im selben WLAN sein.

Lizenz: Alle Rechte vorbehalten. Private Nutzung ist frei, bei öffentlicher Vorstellung sind Credits Pflicht.
Weiterverbreitung und kommerzielle Nutzung nur mit schriftlicher Erlaubnis. Details siehe [LICENSE](LICENSE).
