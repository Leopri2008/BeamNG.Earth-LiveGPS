[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/Q0D12ADPNC)

With that you support the creator of the GPS mod. Not Beamng.earth
# BeamNG.Earth Live GPS

Drive in [BeamNG.Earth](https://github.com/TecnaGamer/BeamNG.Earth), and your **Android phone's GPS follows your car in real time**,
as if you were really driving there. Open Google Maps, Waze or any navigation app on your phone and watch it track
your in-game drive on the real-world map.

**Live map in your browser:** works on any device in your Wi-Fi, **including iPhone and iPad**, plus a stream overlay for OBS.

**Drive recorder (v1.2):** the script records your route and trip even with the browser closed, keeps them until you
reset them, exports them as **GPX or KML**, and keeps a lifetime **odometer** that only counts real driving.

**Fog of the world (v1.3):** the map is covered in a light mist. Every place you really drive through is revealed and stays
revealed. Wrongly revealed cells can be erased again in an erase mode with a grid.

**v2.0 Navigation, POIs and more:**
- The script can read the position straight from BeamNG.Earth's own feed (`--feed`, no launch option needed). Both works
- **Street and town** you are in are shown on the map.
- **Search and turn-by-turn navigation** on the map: places, addresses and categories (fuel, charging, food, fast food, café, parking, shops, pharmacy),
  stops, avoid motorways/tolls/ferries, turn arrows on the route, and the route starts in the direction the car is facing. It works while you drive.
- **Points of interest with real icons**, thinned out by importance and zoom, switchable by group; the panels adapt to phones, tablets and big screens.
- **Search along the route:** only places beside the route ahead of you, loaded piece by piece (36 km ahead), with several map-data servers,
  short timeouts and a **Try again** button instead of endless loading.
- **Place card** with opening hours (open/closed), phone, website, an Wikipedia photo and short text where OpenStreetMap or Wikipedia have them.
- **Fuel prices** (cheapest first, local currency) for Germany (own free key), Austria, France, Spain, Italy, Western Australia, Mexico, Sweden, Denmark,
  Greece, Croatia, Iceland, Slovenia and Romania, with a backup source and the last known prices if a service is down.

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
2. Run `python pc/bne_live_coords[version].py`
   It prints the address for the phone, e.g. `Phone app address:  192.168.178.20:47700`.
   and the live map address, e.g. `Live map (browser): http://192.168.178.20:47700/map`.
   If Windows Firewall asks, allow access on **private networks**.
3. Steam → BeamNG.drive → Properties → Launch options: `-bngeport 47614`
4. Start BeamNG.drive and load BeamNG.Earth. The console shows `Game connected.` and then a `LIVE` line with your coordinates.

To go back to normal, just remove the launch option.

#### Without the launch option: BeamNG.Earth's companion feed

Newer BeamNG.Earth versions publish the car's position themselves, read-only, on `http://127.0.0.1:47615/v1/stream`
(the "companion feed"). With one of those, run the script with `--feed` instead of setting up the relay:

```
python "pc/bne_live_coords v1.3.py" --feed
```

No `-bngeport` launch option is needed, and nothing sits between the game and the BeamNG.Earth server. The script
reconnects by itself when BeamNG.Earth restarts. `BNE_FEED=http://host:port` points it at another address.

`--port <number>` serves the phone app and the live map on another port than 47700 (for example when 47700 is taken);
the phone app's address then ends in that port.

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
- **Trip stats:** distance, drive time, average and top speed, plus your **total km** driven with Live GPS
- **Fog of the world:** see [section 5](#5-fog-of-the-world)
- **Export** the route as GPX (GPS apps) or KML (Google Earth) in the settings, or open `http://PC-IP:47700/export.gpx` / `.kml`
- Settings are remembered per browser. Desktop keys: `C` camera, `B` 3D, `S` style, `R` recenter, `F` fullscreen, `U` units

**iPhone / iPad:** for full screen, tap Share → **Add to Home Screen** and open it from there.

**Screen stays on:** on Android, open the map with the **Open live map** button in the app, which keeps the screen on reliably.
In a browser the page tries to keep the screen on too (tap the map once after opening it), but over the Wi-Fi address
browsers don't guarantee it. If it still turns off (e.g. iPhone), set Auto-Lock to "Never" while driving.

**OBS overlay:** add a Browser Source with `http://127.0.0.1:47700/map?obs=1` (map + speedometer only).
Options: `&stats=1` shows trip stats, `&style=liberty`, `&cam=heading`, `&units=imperial`, `&buildings=0`, `&trail=0`.

### 4. Drive recorder

The script records everything itself, so it also works while the map is closed or the phone is in your pocket:

- **Route and trip** are saved in the folder `BNE_LiveGPS_data` next to the script and stays even at restarts.
  continues where you stopped. Nothing is deleted until you tap **Reset trip & route** (twice, so it never happens by accident).
- **Teleports never draw a line through the world:** a jump starts a new route part. Short glitches are ignored completely.
- **Only real driving counts:** distance only adds up when the car's position moves the way its own speed says, and
  the top speed only counts when the position confirms it for a second. Teleports, glitches and the free camera add nothing.
- **Total km (odometer)** is never reset by the trip reset.

### 5. Fog of the world

The map is covered in a translucent mist. The map is divided into a grid of squares (about 25 m). Only the squares you
really drive through are filled, and they stay filled, across restarts, until you reset them. The mist opens a little wider
around them, with soft edges.

- **Only real driving reveals:** it uses the same check as the odometer. Teleports, glitches and the free camera reveal nothing,
  and the gap between two position updates is filled in, so fast driving leaves no holes.
- **Erase mode** (Settings → Fog of the world → *Erase cells*): the mist is lifted and you see the grid. Revealed cells are
  outlined in orange. Tap a cell to remove it (brush: 1 cell, 7×7 or 17×17), *Undo* brings it back, *Done* leaves the mode.
  Zoom in a bit to edit, the grid appears at street level.
- **Reset fog** (tap twice) clears everything. It is separate from *Reset trip & route* and from the odometer.
- The mist can be switched off in the settings, or in the OBS overlay with `&fog=0`.
- The explored cells are stored on your PC in `BNE_LiveGPS_data/fog.json`. The settings show how many km² you have explored.

### 6. Trips

The **trips button** on the map (or the key `T`) opens a scrollable list of all your drives, newest first, with distance,
drive time, average and top speed, and where it started and ended.

- **A trip lasts until you end it:** it keeps running across restarts of the script and pauses of any length, and ends only
  when you tap *Start new trip* (or reset it). Then it moves to the list and is never overwritten. Drives shorter than 100 m are not kept.
- **Show on map** draws a finished trip in its own colour (several at once are fine), *Zoom to* flies there, *GPX* exports it,
  *Delete* removes it for good (tap twice). *Discard* drops the current trip without saving it.
- Trips are stored in `BNE_LiveGPS_data/trips/`, one file each. The odometer and the fog are not touched by deleting trips.
- When updating from v1.3: your unfinished v1.3 trip simply continues as the current trip.

### 7. Street and town

The map shows the road and town you are driving through, like the header of a navigation app. The script asks
OpenStreetMap's free [Nominatim](https://nominatim.org) service for it: at most one request every 4 seconds, only after you
moved a bit, answers are cached, and a polite User-Agent is sent. **This sends the car's game-world coordinates to
Nominatim.** Turn it off with `--no-places` (the road and town then disappear from the map, the trips lose their place names).

### 8. Search and navigation

The **search button** on the map (or the key `/`) opens search and navigation. Everything works while you drive too. On a phone it
is a sheet at the bottom that leaves the map free; on a tablet or desktop it is a panel on the left. The map stays movable all the time.

- **Smart search.** Type a place, an address or a business. The script asks Nominatim twice, once worldwide and once around the car,
  and ranks the answers by how well known they are **divided by distance**
- **Categories in any language.** Type *Tankstelle*, *tanken*, *gas station*, *essence*, *gasolinera*, *benzina*, *Apotheke*,
  *pharmacy*, *Parkplatz*, *Hotel*, ... (many languages) and the category appears as a suggestion while you type. You can add a place:
  *Gas Station in Hamburg*. The chips under the search field do the same (fuel, charging, food, café, parking, shops, pharmacy,
  hospital, hotels, ATM, bars, toilets, sights).
- **Tiles first.** A category is first looked up in the map tiles that are already loaded around the car (instant, **no request at all**).
  Only when fewer than five are found, or when you tap *Search wider (online)*, Nominatim is asked (one request, 1.1 s apart,
  cached for 7 days). The list is sorted nearest first and every result is a pin with its own icon and name on the map.
- **Route here** plans the route from the car and **shows the whole route** next to the panel, so you see where it goes
  You can **add stops** (search again, tap *Add as stop*: it goes where it
  costs the least detour, so a gas station on the way becomes the next stop), remove or move them, and choose **Avoid motorways / tolls
  / ferries**. The summary shows time, distance and arrival time.
- **Start** shows the next turn with distance at the top, the remaining
  distance, time and arrival. The route button in the bar shows the whole route again. Drive off the route and it **reroutes by
  itself**. Reaching a stop moves on to the next one, reaching the destination ends the navigation. A running navigation survives a
  page reload.
- **Routing** uses the free public [Valhalla](https://valhalla.github.io/valhalla/) server of OpenStreetMap (FOSSGIS), **search**
  uses Nominatim. **Both are shared fair-use services with no guarantee.** Everything goes through the script with a pause between
  requests, a polite User-Agent, and a search cache (`search_cache.json`). **Your search text and the game-world coordinates are
  sent to those servers.** Turn it all off with `--no-nav`, or use your own routing server with `--router http://127.0.0.1:8002/route`.

#### Search along the route, details and fuel prices

With a route planned or running, the category chips search **along the route**: only places beside the route and ahead of you
(up to 36 km ahead, within 1.2 km of the road), sorted by how little they are off your way, with "in 19 km · 90 m off route". The switch above the
chips changes to *Nearby* (around the car). This uses the OpenStreetMap server Overpass (`--no-nav` turns it off).

Tap a place for its card: opening hours with open/closed (judged by the time zone of the place's location),
phone, website, and, for notable places, a photo and a short text from Wikipedia. Ratings and reviews do not exist in free data and are not shown.

**Fuel prices** (live, shown in the list, on the pins and on the card; the list has the fuel type, "Cheapest first" and a **Cheapest** tag):

| Country | Source | Key |
|---|---|---|
| Germany | Tankerkönig (MTS-K, CC BY 4.0) | **your own free key**, see below |
| Austria | E-Control Spritpreisrechner | none |
| France | data.economie.gouv.fr | none |
| Spain | Ministerio para la Transición Ecológica | none |
| Italy | MIMIT open data (daily snapshot of 08:00, not live) | none |
| Western Australia | FuelWatch (CC BY 4.0, price of the day) | none |
| Mexico | CRE (pesos) | none |
| Sweden, Denmark, Greece, Croatia, Iceland, Slovenia, Romania | [fuel-prices.eu](https://www.fuel-prices.eu/api/docs/) (open mirror, EUR, no key *today*) | none |

**Needs its own free key:** only Germany. **Not built in**
Italy, Western Australia and Mexico are built from the published formats and not yet verified against the live servers. Everywhere else the list says that there are no prices.
Features depend on the region: the map, search, routes and symbols work worldwide.

*Key for Germany:* register for free at [creativecommons.tankerkoenig.de](https://creativecommons.tankerkoenig.de) and put the key either into
the environment variable `TANKERKOENIG_KEY` or into the file `keys.json` in the data folder: `{"tankerkoenig": "your-key"}`.
The key stays on your PC. It is never sent to the browser, never written into the script and never uploaded to GitHub (the terms forbid
publishing keys), so nobody can use up your quota through the script; everybody who runs it uses their own key. Tankerkönig allows one
request per minute; the script asks only when you search for fuel and remembers the answer for 5 minutes.

### 9. Points of interest

Settings → *Navigation & map points* switches **groups** on the map from street level: fuel and charging, food and drink, shops,
parking, hotels and camping, health, banks/post/toilets, sights and parks. Each point has its own **icon in the colour of its kind**
(a pump for gas stations, fork and knife for restaurants, a bed for hotels, ...), the name shows when you zoom in. Charging stations, toilets, doctors and similar appear only when you are zoomed in close. Tap one to route there or add it as a stop.
The default *Avoid …* options are in the same settings section.

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
- On start the script looks **once** for a newer release on GitHub (no personal data is sent, nothing is downloaded or installed)
- The HTTP endpoint is reachable by anyone on your local network (read-only, position only).
- Your route and stats are stored only on your PC, in `BNE_LiveGPS_data` next to the script. Delete that folder to start completely fresh (this also resets the total km).
- Map data © [OpenFreeMap](https://openfreemap.org), © [OpenMapTiles](https://www.openmaptiles.org/), data from [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors.
  
## Credits

- [BeamNG.Earth](https://github.com/TecnaGamer/BeamNG.Earth) by TecnaGamer. This is an unofficial companion tool,
  not affiliated with BeamNG.Earth or BeamNG GmbH.

## License

**All rights reserved.** Free for personal use; credit required when shown publicly.
Re-uploading, redistributing modified or unmodified copies, and commercial use are **not allowed** without
written permission. Please share a link to this repository instead. Full terms: [LICENSE](LICENSE).

If you show it in a video or stream, please credit:
`BeamNG.Earth Live GPS by Leopri2008 – https://github.com/Leopri2008/BeamNG.Earth-LiveGPS`
