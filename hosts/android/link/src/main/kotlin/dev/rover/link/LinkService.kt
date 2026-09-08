package dev.rover.link

import android.app.Service
import android.content.Intent
import android.os.IBinder

/**
 * The rover link's foreground service.  A stub: it does nothing yet.
 *
 * It exists so the manifest can pin the link to its own process (`:link`) from
 * the first commit.  The serial transport, the 20 Hz command stream, the
 * validator and the bus arrive after the Pi host passes G4 on hardware; see
 * README.md.
 */
class LinkService : Service() {
    override fun onBind(intent: Intent?): IBinder? = null
}
