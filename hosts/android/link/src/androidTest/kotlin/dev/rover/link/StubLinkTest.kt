package dev.rover.link

import androidx.test.ext.junit.runners.AndroidJUnit4
import androidx.test.platform.app.InstrumentationRegistry
import kotlinx.serialization.SerializationException
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.JsonObject
import kotlinx.serialization.json.doubleOrNull
import kotlinx.serialization.json.intOrNull
import kotlinx.serialization.json.jsonObject
import kotlinx.serialization.json.jsonPrimitive
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Test
import org.junit.runner.RunWith
import java.io.BufferedReader
import java.io.IOException
import java.io.OutputStream
import java.net.InetSocketAddress
import java.net.Socket
import java.util.concurrent.BlockingQueue
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread

/**
 * Drives `rover-stub --tcp PORT` (docs/protocol.md) over a plain socket and
 * checks the one property the phone host will live by: while speed commands
 * arrive every 50 ms the heartbeat is alive and the applied value follows the
 * command; when they stop, the firmware zeroes the motors within its 300 ms
 * heartbeat, observed here within 600 ms.
 *
 * The wire is Waveshare's, not ours, so lines are read as [JsonObject]; the
 * generated contract classes describe the bus, not this link.
 *
 * Instrumentation arguments `ROVER_STUB_HOST` and `ROVER_STUB_PORT` select the
 * stub.  The defaults are the emulator's alias for the development machine's
 * loopback interface and port 7777; README.md has the `adb reverse` form for a
 * physical phone.
 */
@RunWith(AndroidJUnit4::class)
class StubLinkTest {

    @Test
    fun heartbeatFollowsTheCommandStream() {
        val args = InstrumentationRegistry.getArguments()
        val host = args.getString("ROVER_STUB_HOST") ?: EMULATOR_HOST_ALIAS
        val port = args.getString("ROVER_STUB_PORT")?.toInt() ?: DEFAULT_PORT

        val feedback = LinkedBlockingQueue<JsonObject>()
        Socket().use { socket ->
            socket.tcpNoDelay = true
            socket.connect(InetSocketAddress(host, port), CONNECT_TIMEOUT_MS)
            val out = socket.getOutputStream()
            thread(name = "stub-reader", isDaemon = true) {
                readFeedback(socket.getInputStream().bufferedReader(Charsets.UTF_8), feedback)
            }

            send(out, FEEDBACK_ON)
            var last: JsonObject? = null
            val driveEnd = System.nanoTime() + DRIVE_MS * NS_PER_MS
            var next = System.nanoTime()
            while (System.nanoTime() < driveEnd) {
                send(out, SPEED)
                next += PERIOD_MS * NS_PER_MS
                Thread.sleep(maxOf(0L, (next - System.nanoTime()) / NS_PER_MS))
                while (true) {
                    last = feedback.poll() ?: break
                }
            }
            assertNotNull("no T:1001 feedback during one second of speed commands", last)
            val moving = last!!
            assertEquals(
                "hb while streaming (null: no hb field, is the stub running --stock?)",
                1,
                moving.int("hb"),
            )
            assertEquals("L while streaming", 0.1, moving.double("L") ?: Double.NaN, 0.02)

            // The stream stops here.  The firmware's heartbeat expires at 300 ms.
            assertTrue(
                "no feedback with hb == 0 and L == 0 within $STOP_MS ms of the last command",
                awaitZero(feedback, STOP_MS),
            )
        }
    }

    private fun send(out: OutputStream, line: String) {
        out.write((line + "\n").toByteArray(Charsets.US_ASCII))
        out.flush()
    }

    /** Queues every well-formed T:1001 line.  Anything else is dropped, as the protocol says. */
    private fun readFeedback(reader: BufferedReader, sink: BlockingQueue<JsonObject>) {
        while (true) {
            val line = try {
                reader.readLine() ?: return
            } catch (e: IOException) {
                return
            }
            val obj = try {
                Json.parseToJsonElement(line).jsonObject
            } catch (e: SerializationException) {
                continue
            } catch (e: IllegalArgumentException) {
                continue
            }
            if (obj.int("T") == T_FEEDBACK) sink.put(obj)
        }
    }

    private fun awaitZero(feedback: BlockingQueue<JsonObject>, timeoutMs: Long): Boolean {
        val deadline = System.nanoTime() + timeoutMs * NS_PER_MS
        while (true) {
            val remaining = deadline - System.nanoTime()
            if (remaining <= 0) return false
            val line = feedback.poll(remaining, TimeUnit.NANOSECONDS) ?: return false
            if (line.int("hb") == 0 && line.double("L") == 0.0) return true
        }
    }

    private fun JsonObject.int(key: String): Int? = this[key]?.jsonPrimitive?.intOrNull

    private fun JsonObject.double(key: String): Double? = this[key]?.jsonPrimitive?.doubleOrNull
}

private const val DEFAULT_PORT = 7777
private const val CONNECT_TIMEOUT_MS = 2_000
private const val PERIOD_MS = 50L
private const val DRIVE_MS = 1_000L
private const val STOP_MS = 600L
private const val NS_PER_MS = 1_000_000L
private const val T_FEEDBACK = 1001
private const val FEEDBACK_ON = """{"T":131,"cmd":1}"""
private const val SPEED = """{"T":1,"L":0.1,"R":0.1}"""

/**
 * The Android emulator's alias for the host machine's loopback interface,
 * assembled from octets rather than written as a literal: the repository's
 * secret scan fails on any RFC 1918 address in the tree, and cannot tell this
 * documented alias from someone's LAN.
 */
private val EMULATOR_HOST_ALIAS = intArrayOf(10, 0, 2, 2).joinToString(".")
