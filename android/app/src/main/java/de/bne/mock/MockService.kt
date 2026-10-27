package de.bne.mock

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.location.Location
import android.location.LocationManager
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import android.os.SystemClock
import androidx.core.app.NotificationCompat
import androidx.core.app.ServiceCompat
import org.json.JSONObject
import java.net.HttpURLConnection
import java.net.URL
import java.util.Locale
import kotlin.math.cos

/**
 * Polls the live position from the PC (pc/bne_live_coords.py, port 47700) every 250 ms and feeds it
 * into the GPS, network and fused providers as a mock location every 200 ms.
 * Between two poses (~2 Hz) the position is extrapolated with the velocity for smooth movement.
 */
class MockService : Service() {

    enum class State { STOPPED, CONNECTING, WAITING, LIVE, STALE, ERROR }

    companion object {
        const val EXTRA_HOST = "host"
        private const val CHANNEL = "mock"
        private const val NOTIF_ID = 1
        private const val PUSH_MS = 200L
        private const val POLL_MS = 250L
        private const val EXTRAPOLATE_MAX_S = 1.0     // no new pose for longer: stop "driving" ahead
        private const val STALE_S = 5.0

        // Read by the UI (MainActivity polls these twice per second)
        @Volatile var running = false
        @Volatile var state = State.STOPPED
        @Volatile var detail = ""
        @Volatile var lat = 0.0
        @Volatile var lon = 0.0
        @Volatile var speedKmh = 0.0
        @Volatile var bearing = 0f
    }

    private class Sample(
        val lat: Double, val lon: Double, val ve: Double, val vn: Double,
        val speed: Float, val bearing: Float, val receivedAt: Long
    )

    @Volatile private var sample: Sample? = null
    private var lastBearing = 0f
    private lateinit var lm: LocationManager
    private var wakeLock: PowerManager.WakeLock? = null
    private val providers = mutableListOf<String>()
    private var threads = listOf<Thread>()

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        if (running) return START_STICKY
        startAsForeground("Starting…")
        val host = intent?.getStringExtra(EXTRA_HOST) ?: run { stopSelf(); return START_NOT_STICKY }
        lm = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        setState(State.CONNECTING, "Connecting to $host…")

        if (!setupProviders()) {
            setState(
                State.ERROR,
                "This app is not selected as mock location app.\n" +
                    "Developer options → Select mock location app → BNE Mock GPS"
            )
            stopSelf()
            return START_NOT_STICKY
        }

        wakeLock = (getSystemService(Context.POWER_SERVICE) as PowerManager)
            .newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "bnemock:run").apply { acquire() }

        running = true
        threads = listOf(
            Thread({ pollLoop("http://$host/") }, "poll"),
            Thread({ pushLoop() }, "push")
        )
        threads.forEach { it.start() }
        return START_STICKY
    }

    private fun setState(s: State, text: String) {
        state = s
        detail = text
    }

    private fun startAsForeground(text: String) {
        val nm = getSystemService(NotificationManager::class.java)
        if (Build.VERSION.SDK_INT >= 26) {
            nm.createNotificationChannel(NotificationChannel(CHANNEL, "Mock GPS", NotificationManager.IMPORTANCE_LOW))
        }
        ServiceCompat.startForeground(
            this, NOTIF_ID, notification(text),
            if (Build.VERSION.SDK_INT >= 29) ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION else 0
        )
    }

    private fun notification(text: String): Notification {
        val open = PendingIntent.getActivity(
            this, 0, Intent(this, MainActivity::class.java), PendingIntent.FLAG_IMMUTABLE
        )
        return NotificationCompat.Builder(this, CHANNEL)
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setContentTitle("BNE Mock GPS active")
            .setContentText(text)
            .setContentIntent(open)
            .setOngoing(true)
            .setOnlyAlertOnce(true)
            .build()
    }

    private fun updateNotification(text: String) {
        getSystemService(NotificationManager::class.java).notify(NOTIF_ID, notification(text))
    }

    /** Registers GPS + network (+ fused on Android 12+) as test providers. false = no mock permission. */
    private fun setupProviders(): Boolean {
        val names = mutableListOf(LocationManager.GPS_PROVIDER, LocationManager.NETWORK_PROVIDER)
        if (Build.VERSION.SDK_INT >= 31) names.add(LocationManager.FUSED_PROVIDER)
        for (name in names) {
            try { lm.removeTestProvider(name) } catch (_: Exception) {}
            try {
                @Suppress("DEPRECATION")
                lm.addTestProvider(
                    name, false, name == LocationManager.GPS_PROVIDER, false, false,
                    true, true, true, 1 /* POWER_LOW */, 1 /* ACCURACY_FINE */
                )
                lm.setTestProviderEnabled(name, true)
                providers.add(name)
            } catch (e: SecurityException) {
                return false
            } catch (_: Exception) {
                // provider cannot be mocked on this device (e.g. fused on some ROMs) -> skip
            }
        }
        return providers.isNotEmpty()
    }

    private fun pollLoop(url: String) {
        var lastT = -1.0
        var fails = 0
        while (running) {
            try {
                val c = URL(url).openConnection() as HttpURLConnection
                c.connectTimeout = 1500
                c.readTimeout = 1500
                val body = c.inputStream.bufferedReader().use { it.readText() }
                c.disconnect()
                fails = 0
                val j = JSONObject(body)
                if (j.has("lat") && j.has("t")) {
                    val t = j.getDouble("t")
                    if (t != lastT) {
                        lastT = t
                        sample = Sample(
                            j.getDouble("lat"), j.getDouble("lon"),
                            j.optDouble("ve", 0.0), j.optDouble("vn", 0.0),
                            j.optDouble("speed_ms", 0.0).toFloat(),
                            j.optDouble("bearing", 0.0).toFloat(),
                            SystemClock.elapsedRealtime()
                        )
                    }
                } else if (sample == null) {
                    setState(State.WAITING, "PC connected. Waiting for a vehicle position from BeamNG…")
                }
            } catch (e: Exception) {
                fails++
                if (fails >= 4) {
                    setState(State.ERROR, "PC not reachable at $url\n${e.javaClass.simpleName}: ${e.message}")
                }
            }
            if (!nap(POLL_MS)) break
        }
    }

    private fun pushLoop() {
        var tick = 0
        while (running) {
            val s = sample
            if (s != null) {
                val age = (SystemClock.elapsedRealtime() - s.receivedAt) / 1000.0
                val dt = age.coerceAtMost(EXTRAPOLATE_MAX_S)
                val stale = age > STALE_S
                val la = s.lat + s.vn * dt / 111_320.0
                val lo = s.lon + s.ve * dt / (111_320.0 * cos(Math.toRadians(s.lat)))
                val speed = if (stale) 0f else s.speed
                if (speed > 0.5f) lastBearing = s.bearing
                push(la, lo, speed, lastBearing)

                lat = la; lon = lo; speedKmh = speed * 3.6; bearing = lastBearing
                if (state != State.ERROR || age < STALE_S) {
                    if (stale) setState(State.STALE, "No new data for ${age.toInt()} s (game paused?)")
                    else setState(State.LIVE, "")
                }
                if (tick % 5 == 0) {
                    updateNotification(
                        if (stale) detail
                        else String.format(Locale.US, "%.6f, %.6f · %.0f km/h", la, lo, speed * 3.6)
                    )
                }
            }
            tick++
            if (!nap(PUSH_MS)) break
        }
    }

    /** Sleeps; false when the service interrupts the thread on stop. */
    private fun nap(ms: Long): Boolean = try { Thread.sleep(ms); running } catch (_: InterruptedException) { false }

    private fun push(lat: Double, lon: Double, speed: Float, bearing: Float) {
        for (p in providers) {
            val loc = Location(p).apply {
                latitude = lat
                longitude = lon
                this.speed = speed
                this.bearing = bearing
                accuracy = 3f
                time = System.currentTimeMillis()
                elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos()
                if (Build.VERSION.SDK_INT >= 26) {
                    speedAccuracyMetersPerSecond = 0.5f
                    bearingAccuracyDegrees = 5f
                    verticalAccuracyMeters = 5f
                }
            }
            try { lm.setTestProviderLocation(p, loc) } catch (e: Exception) {
                setState(State.ERROR, "Could not set mock location: ${e.message}")
            }
        }
    }

    override fun onDestroy() {
        running = false
        threads.forEach { it.interrupt() }
        if (::lm.isInitialized) for (p in providers) {
            try { lm.setTestProviderEnabled(p, false) } catch (_: Exception) {}
            try { lm.removeTestProvider(p) } catch (_: Exception) {}
        }
        providers.clear()
        wakeLock?.let { if (it.isHeld) it.release() }
        if (state != State.ERROR) setState(State.STOPPED, "")
        super.onDestroy()
    }
}
