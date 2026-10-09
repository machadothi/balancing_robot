package com.machadothi.balancebot

import com.machadothi.balancebot.data.camera.MjpegReader
import org.junit.Assert.assertArrayEquals
import org.junit.Test
import java.io.ByteArrayInputStream
import java.io.EOFException

class CameraTest {

    private fun part(data: ByteArray) =
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ${data.size}\r\n\r\n".toByteArray() +
            data + "\r\n".toByteArray()

    @Test
    fun framesAreSplitByContentLength() {
        // Frame bytes may contain CR/LF and "--frame": only Content-Length counts
        val a = byteArrayOf(0xFF.toByte(), 0xD8.toByte(), 13, 10, 0xFF.toByte(), 0xD9.toByte())
        val b = "--frame\r\n\r\nxyz".toByteArray()
        val reader = MjpegReader(ByteArrayInputStream(part(a) + part(b)))
        assertArrayEquals(a, reader.nextFrame())
        assertArrayEquals(b, reader.nextFrame())
    }

    @Test(expected = EOFException::class)
    fun truncatedFrameEnds() {
        val whole = part(ByteArray(100) { it.toByte() })
        MjpegReader(ByteArrayInputStream(whole.copyOf(80))).nextFrame()
    }
}

class WifiScanTest {

    @Test
    fun scanListKeepsSpacesInNames() {
        val list = com.machadothi.balancebot.model.WifiNetwork.parseList("-48 home net\t-71 neighbour\tjunk\t-90 ")
        org.junit.Assert.assertEquals(
            listOf(
                com.machadothi.balancebot.model.WifiNetwork("home net", -48),
                com.machadothi.balancebot.model.WifiNetwork("neighbour", -71),
            ),
            list,
        )
        org.junit.Assert.assertEquals(4, list[0].bars)
        org.junit.Assert.assertEquals(2, list[1].bars)
    }

    @Test
    fun emptyScan() {
        org.junit.Assert.assertEquals(emptyList<Any>(), com.machadothi.balancebot.model.WifiNetwork.parseList(""))
    }
}
