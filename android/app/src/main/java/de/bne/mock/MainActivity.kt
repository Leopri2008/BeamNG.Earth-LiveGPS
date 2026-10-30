package de.bne.mock

import android.Manifest
import android.app.Activity
import android.content.ClipData
import android.content.ClipboardManager
import android.content.Intent
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.Color
import android.graphics.Typeface
import android.graphics.drawable.GradientDrawable
import android.graphics.drawable.RippleDrawable
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.provider.Settings
import android.text.InputType
import android.view.Gravity
import android.view.View
import android.view.WindowInsets
import android.view.inputmethod.EditorInfo
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import java.util.Locale

class MainActivity : Activity() {

    // Palette
    private val bg = Color.parseColor("#0D1015")
    private val card = Color.parseColor("#161B22")
    private val field = Color.parseColor("#0F1318")
    private val stroke = Color.parseColor("#262D37")
    private val text1 = Color.parseColor("#E8ECF1")
    private val text2 = Color.parseColor("#8B95A3")
    private val accent = Color.parseColor("#FF7A1A")
    private val green = Color.parseColor("#3DDC84")
    private val amber = Color.parseColor("#F5B841")
    private val red = Color.parseColor("#FF5C5C")

    private lateinit var hostField: EditText
    private lateinit var dot: View
    private lateinit var stateLabel: TextView
    private lateinit var coordsView: TextView
    private lateinit var speedView: TextView
    private lateinit var headingView: TextView
    private lateinit var detailView: TextView
    private lateinit var mainButton: TextView

    private val ui = Handler(Looper.getMainLooper())
    private val prefs by lazy { getSharedPreferences("cfg", MODE_PRIVATE) }

    private fun dp(v: Int) = (v * resources.displayMetrics.density).toInt()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.decorView.setBackgroundColor(bg)

        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(20), dp(20), dp(20), dp(24))
        }

        // Header
        root.addView(label("BNE Mock GPS", 28f, text1, bold = true))
        root.addView(label("Live GPS from BeamNG.Earth", 14f, text2).apply { setPadding(0, dp(2), 0, dp(20)) })

        // Status card
        val status = cardLayout()
        val stateRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
        dot = View(this).apply { background = circle(text2) }
        stateRow.addView(dot, LinearLayout.LayoutParams(dp(10), dp(10)).apply { rightMargin = dp(8) })
        stateLabel = label("STOPPED", 13f, text2, bold = true).apply { letterSpacing = 0.12f }
        stateRow.addView(stateLabel)
        status.addView(stateRow)

        coordsView = label("—", 22f, text1, mono = true).apply {
            setPadding(0, dp(14), 0, dp(4))
            setOnLongClickListener { copyCoords(); true }
        }
        status.addView(coordsView)

        val metrics = LinearLayout(this).apply { orientation = LinearLayout.HORIZONTAL }
        speedView = label("0 km/h", 16f, text2, mono = true)
        headingView = label("", 16f, text2, mono = true).apply { gravity = Gravity.END }
        metrics.addView(speedView, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
        metrics.addView(headingView, LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f))
        status.addView(metrics)

        detailView = label("", 13f, text2).apply { setPadding(0, dp(10), 0, 0); visibility = View.GONE }
        status.addView(detailView)
        root.addView(status, cardParams())

        // Connection card
        val conn = cardLayout()
        conn.addView(label("PC ADDRESS", 12f, text2, bold = true).apply { letterSpacing = 0.12f })
        hostField = EditText(this).apply {
            setText(prefs.getString("host", ""))
            hint = "192.168.x.x:47700"
            setTextColor(text1)
            setHintTextColor(Color.parseColor("#4A5361"))
            typeface = Typeface.MONOSPACE
            textSize = 17f
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_URI
            imeOptions = EditorInfo.IME_ACTION_DONE
            isSingleLine = true
            background = rounded(field, dp(12), stroke)
            setPadding(dp(14), dp(12), dp(14), dp(12))
        }
        conn.addView(hostField, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT
        ).apply { topMargin = dp(10) })
        conn.addView(label("Shown in the console of bne_live_coords.py on your PC.", 12f, text2)
            .apply { setPadding(dp(2), dp(8), 0, dp(16)) })

        mainButton = button("START", accent, Color.parseColor("#1A0E05")) { toggle() }
        conn.addView(mainButton, LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT, dp(52)
        ))
        conn.addView(button("Open live map", card, accent, outline = true) { openLiveMap() },
            LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(46)).apply { topMargin = dp(10) })
        root.addView(conn, cardParams())

        // Setup help
        val help = cardLayout()
        help.addView(label("FIRST-TIME SETUP", 12f, text2, bold = true).apply { letterSpacing = 0.12f })
        help.addView(label(
            "1.  Enable Developer options (Settings → About phone → tap Build number 7×)\n" +
                "2.  Developer options → Select mock location app → BNE Mock GPS\n" +
                "3.  PC and phone on the same Wi-Fi\n" +
                "4.  Enter the PC address and tap START",
            14f, text1
        ).apply { setPadding(0, dp(10), 0, dp(14)); setLineSpacing(dp(4).toFloat(), 1f) })
        help.addView(button("Open developer options", card, accent, outline = true) { openDevOptions() },
            LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, dp(46)))
        root.addView(help, cardParams())

        root.addView(label("Long-press the coordinates to copy them.", 12f, text2).apply {
            gravity = Gravity.CENTER
            setPadding(0, dp(8), 0, 0)
        })

        // Android 15 draws edge-to-edge: keep content clear of status/navigation bars and keyboard
        val scroll = ScrollView(this).apply {
            setBackgroundColor(bg)
            isFillViewport = true
            addView(root)
        }
        scroll.setOnApplyWindowInsetsListener { v, insets ->
            if (Build.VERSION.SDK_INT >= 30) {
                val b = insets.getInsets(WindowInsets.Type.systemBars() or WindowInsets.Type.ime())
                v.setPadding(0, b.top, 0, b.bottom)
            } else {
                v.setPadding(0, insets.systemWindowInsetTop, 0, insets.systemWindowInsetBottom)
            }
            insets
        }
        setContentView(scroll)

        askPermissions()
    }

    // ---------- actions ----------

    private fun toggle() {
        if (MockService.running) {
            stopService(Intent(this, MockService::class.java))
        } else {
            start()
        }
    }

    private fun start() {
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            MockService.state = MockService.State.ERROR
            MockService.detail = "Location permission missing. Please allow it."
            askPermissions()
            return
        }
        val host = hostField.text.toString().trim()
            .removePrefix("http://").removeSuffix("/")
        if (host.isEmpty()) {
            Toast.makeText(this, "Enter the PC address first", Toast.LENGTH_SHORT).show()
            return
        }
        prefs.edit().putString("host", host).apply()
        hostField.clearFocus()
        val i = Intent(this, MockService::class.java).putExtra(MockService.EXTRA_HOST, host)
        if (Build.VERSION.SDK_INT >= 26) startForegroundService(i) else startService(i)
    }

    private fun openLiveMap() {
        val host = hostField.text.toString().trim().removePrefix("http://").removeSuffix("/")
        if (host.isEmpty()) {
            Toast.makeText(this, "Enter the PC address first", Toast.LENGTH_SHORT).show()
            return
        }
        prefs.edit().putString("host", host).apply()
        // in-app map: the screen stays on (a browser tab over Wi-Fi can't guarantee that)
        startActivity(Intent(this, MapActivity::class.java).putExtra(MapActivity.EXTRA_URL, "http://$host/map"))
    }

    private fun openDevOptions() {
        try {
            startActivity(Intent(Settings.ACTION_APPLICATION_DEVELOPMENT_SETTINGS))
        } catch (_: Exception) {
            startActivity(Intent(Settings.ACTION_SETTINGS))
        }
    }

    private fun copyCoords() {
        if (MockService.state != MockService.State.LIVE && MockService.state != MockService.State.STALE) return
        val txt = String.format(Locale.US, "%.6f, %.6f", MockService.lat, MockService.lon)
        getSystemService(ClipboardManager::class.java).setPrimaryClip(ClipData.newPlainText("coordinates", txt))
        Toast.makeText(this, "Copied $txt", Toast.LENGTH_SHORT).show()
    }

    private fun askPermissions() {
        val need = mutableListOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION)
        if (Build.VERSION.SDK_INT >= 33) need.add(Manifest.permission.POST_NOTIFICATIONS)
        val missing = need.filter { checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED }
        if (missing.isNotEmpty()) requestPermissions(missing.toTypedArray(), 1)
    }

    // ---------- live refresh ----------

    private val refresh = object : Runnable {
        override fun run() {
            render()
            ui.postDelayed(this, 250)
        }
    }

    private fun render() {
        val st = MockService.state
        val (name, color) = when (st) {
            MockService.State.STOPPED -> "STOPPED" to text2
            MockService.State.CONNECTING -> "CONNECTING" to amber
            MockService.State.WAITING -> "WAITING FOR GAME" to amber
            MockService.State.LIVE -> "LIVE" to green
            MockService.State.STALE -> "NO NEW DATA" to amber
            MockService.State.ERROR -> "ERROR" to red
        }
        stateLabel.text = name
        stateLabel.setTextColor(color)
        (dot.background as GradientDrawable).setColor(color)

        val hasPos = st == MockService.State.LIVE || st == MockService.State.STALE
        if (hasPos) {
            coordsView.text = String.format(Locale.US, "%.6f\n%.6f", MockService.lat, MockService.lon)
            speedView.text = String.format(Locale.US, "%.0f km/h", MockService.speedKmh)
            headingView.text = String.format(Locale.US, "%s %03.0f°", compass(MockService.bearing), MockService.bearing)
            coordsView.setTextColor(text1)
        } else {
            coordsView.text = "—"
            speedView.text = ""
            headingView.text = ""
            coordsView.setTextColor(text2)
        }

        val d = MockService.detail
        detailView.text = d
        detailView.setTextColor(if (st == MockService.State.ERROR) red else text2)
        detailView.visibility = if (d.isEmpty()) View.GONE else View.VISIBLE

        val running = MockService.running
        if (mainButton.tag != running) {
            mainButton.tag = running
            if (running) styleButton(mainButton, "STOP", card, red, outline = true)
            else styleButton(mainButton, "START", accent, Color.parseColor("#1A0E05"))
            hostField.isEnabled = !running
            hostField.alpha = if (running) 0.6f else 1f
        }
    }

    private fun compass(deg: Float): String {
        val dirs = arrayOf("N", "NE", "E", "SE", "S", "SW", "W", "NW")
        return dirs[(((deg % 360) + 360) % 360 / 45f + 0.5f).toInt() % 8]
    }

    override fun onResume() { super.onResume(); ui.post(refresh) }
    override fun onPause() { ui.removeCallbacks(refresh); super.onPause() }

    // ---------- view helpers ----------

    private fun label(t: String, size: Float, color: Int, bold: Boolean = false, mono: Boolean = false) =
        TextView(this).apply {
            text = t
            textSize = size
            setTextColor(color)
            typeface = when {
                mono && bold -> Typeface.create(Typeface.MONOSPACE, Typeface.BOLD)
                mono -> Typeface.MONOSPACE
                bold -> Typeface.create("sans-serif-medium", Typeface.BOLD)
                else -> Typeface.create("sans-serif", Typeface.NORMAL)
            }
        }

    private fun rounded(color: Int, radius: Int, strokeColor: Int? = null) = GradientDrawable().apply {
        setColor(color)
        cornerRadius = radius.toFloat()
        if (strokeColor != null) setStroke(dp(1), strokeColor)
    }

    private fun circle(color: Int) = GradientDrawable().apply {
        shape = GradientDrawable.OVAL
        setColor(color)
    }

    private fun cardLayout() = LinearLayout(this).apply {
        orientation = LinearLayout.VERTICAL
        background = rounded(card, dp(20), stroke)
        setPadding(dp(18), dp(18), dp(18), dp(18))
    }

    private fun cardParams() = LinearLayout.LayoutParams(
        LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT
    ).apply { bottomMargin = dp(14) }

    private fun button(t: String, fill: Int, fg: Int, outline: Boolean = false, onClick: () -> Unit) =
        TextView(this).apply {
            styleButton(this, t, fill, fg, outline)
            gravity = Gravity.CENTER
            isClickable = true
            isFocusable = true
            setOnClickListener { onClick() }
        }

    private fun styleButton(b: TextView, t: String, fill: Int, fg: Int, outline: Boolean = false) {
        b.text = t
        b.textSize = 15f
        b.letterSpacing = 0.08f
        b.typeface = Typeface.create("sans-serif-medium", Typeface.BOLD)
        b.setTextColor(fg)
        val shape = rounded(fill, dp(14), if (outline) fg else null)
        b.background = RippleDrawable(ColorStateList.valueOf(Color.argb(60, 255, 255, 255)), shape, null)
    }
}
