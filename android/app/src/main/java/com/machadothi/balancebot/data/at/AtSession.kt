package com.machadothi.balancebot.data.at

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeoutOrNull
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream

/**
 * One command at a time over a byte stream: write "AT...\r", wait for its reply.
 *
 * A reader coroutine splits the input into lines. [transact] holds a mutex, so the
 * polling loop, the jog and the settings screen can share the link safely.
 */
class AtSession(
    private val input: InputStream,
    private val output: OutputStream,
    scope: CoroutineScope,
) {
    private val lines = Channel<String>(capacity = 64, onBufferOverflow = BufferOverflow.DROP_OLDEST)
    private val mutex = Mutex()

    private val reader = scope.launch(Dispatchers.IO) {
        val buffer = StringBuilder()
        val bytes = ByteArray(256)
        try {
            while (isActive) {
                val n = input.read(bytes)
                if (n < 0) break
                for (i in 0 until n) {
                    val c = bytes[i].toInt().toChar()
                    if (c == '\n') {
                        AtProtocol.clean(buffer.toString())?.let { lines.trySend(it) }
                        buffer.clear()
                    } else {
                        buffer.append(c)
                    }
                }
            }
        } catch (ignored: IOException) {
            // The link was closed: transact() reports it
        } finally {
            lines.close()
        }
    }

    val isOpen: Boolean get() = reader.isActive

    /** Send one command and wait for its final reply line */
    suspend fun transact(command: String, timeoutMs: Long = DEFAULT_TIMEOUT_MS): AtReply = mutex.withLock {
        // Lines left over from a reply that timed out belong to an older command
        while (lines.tryReceive().isSuccess) Unit

        withContext(Dispatchers.IO) {
            output.write((command + "\r").toByteArray())
            output.flush()
        }
        withTimeoutOrNull(timeoutMs) { awaitReply() } ?: AtReply.Timeout
    }

    private suspend fun awaitReply(): AtReply {
        var reply: AtReply? = null
        while (reply == null) {
            val line = lines.receiveCatching().getOrNull() ?: throw IOException("link closed")
            reply = AtProtocol.parseFinal(line)
        }
        return reply
    }

    companion object {
        /** A reply over the 9600 baud Bluetooth link takes ~50 ms */
        const val DEFAULT_TIMEOUT_MS = 1500L
    }
}
