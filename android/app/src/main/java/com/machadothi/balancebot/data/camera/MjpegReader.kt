package com.machadothi.balancebot.data.camera

import java.io.EOFException
import java.io.IOException
import java.io.InputStream

/**
 * Splits a multipart/x-mixed-replace (MJPEG) body into JPEG frames. Every part
 * carries a Content-Length header, as the Atom's /stream sends it
 * (atom/main/camera_http.c).
 */
class MjpegReader(private val input: InputStream) {

    /** The next frame's JPEG bytes; throws EOFException at the end of the stream */
    fun nextFrame(): ByteArray {
        var length = -1
        // Part headers: skip the boundary and blank lines, stop at the blank line after the headers
        while (true) {
            val line = readLine()
            if (line.isEmpty()) {
                if (length >= 0) break
                continue
            }
            val colon = line.indexOf(':')
            if (colon > 0 && line.substring(0, colon).trim().equals("Content-Length", ignoreCase = true)) {
                length = line.substring(colon + 1).trim().toIntOrNull()
                    ?: throw IOException("bad Content-Length: $line")
                if (length !in 1..MAX_FRAME) throw IOException("frame size $length out of range")
            }
        }
        val frame = ByteArray(length)
        var read = 0
        while (read < length) {
            val n = input.read(frame, read, length - read)
            if (n < 0) throw EOFException()
            read += n
        }
        return frame
    }

    private fun readLine(): String {
        val line = StringBuilder()
        while (true) {
            val c = input.read()
            if (c < 0) throw EOFException()
            if (c == '\n'.code) return line.toString().trimEnd('\r')
            if (line.length < MAX_LINE) line.append(c.toChar())
        }
    }

    private companion object {
        const val MAX_FRAME = 2_000_000
        const val MAX_LINE = 256
    }
}
