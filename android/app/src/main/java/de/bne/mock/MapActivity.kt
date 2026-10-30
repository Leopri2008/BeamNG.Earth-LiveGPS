package de.bne.mock

import android.annotation.SuppressLint
import android.app.Activity
import android.graphics.Color
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.view.View
import android.view.WindowInsets
import android.view.WindowInsetsController
import android.view.WindowManager
import android.webkit.WebChromeClient
import android.webkit.WebResourceRequest
import android.webkit.WebView
import android.webkit.WebViewClient

/**
 * Shows the live map (http://PC:47700/map) full screen inside the app.
 * Unlike a browser tab, the app can keep the screen on reliably (FLAG_KEEP_SCREEN_ON).
 */
class MapActivity : Activity() {

    companion object {
        const val EXTRA_URL = "url"
    }

    private lateinit var web: WebView

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        window.decorView.setBackgroundColor(Color.parseColor("#0D1015"))

        web = WebView(this).apply {
            setBackgroundColor(Color.parseColor("#0D1015"))
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true            // map remembers its settings
            settings.mediaPlaybackRequiresUserGesture = false
            settings.loadWithOverviewMode = true
            settings.useWideViewPort = true
            webChromeClient = WebChromeClient()
            webViewClient = object : WebViewClient() {
                // map links (OpenStreetMap credits etc.) open in the normal browser
                override fun shouldOverrideUrlLoading(view: WebView, request: WebResourceRequest): Boolean {
                    val url = request.url
                    return if (url.path?.startsWith("/map") == true && url.host == Uri.parse(startUrl).host) {
                        false
                    } else {
                        try { startActivity(android.content.Intent(android.content.Intent.ACTION_VIEW, url)) } catch (_: Exception) {}
                        true
                    }
                }
            }
        }
        setContentView(web)
        goFullscreen()
        if (savedInstanceState != null) web.restoreState(savedInstanceState) else web.loadUrl(startUrl)
    }

    private val startUrl: String
        get() = intent.getStringExtra(EXTRA_URL) ?: "about:blank"

    private fun goFullscreen() {
        if (Build.VERSION.SDK_INT >= 30) {
            window.insetsController?.let {
                it.hide(WindowInsets.Type.systemBars())
                it.systemBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE
            }
        } else {
            @Suppress("DEPRECATION")
            window.decorView.systemUiVisibility = (View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                or View.SYSTEM_UI_FLAG_FULLSCREEN or View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                or View.SYSTEM_UI_FLAG_LAYOUT_STABLE)
        }
    }

    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        if (hasFocus) goFullscreen()
    }

    override fun onSaveInstanceState(outState: Bundle) {
        super.onSaveInstanceState(outState)
        web.saveState(outState)
    }

    override fun onResume() { super.onResume(); web.onResume() }
    override fun onPause() { web.onPause(); super.onPause() }

    override fun onDestroy() {
        web.destroy()
        super.onDestroy()
    }
}
