package com.machadothi.balancebot.data.link

import android.bluetooth.BluetoothSocket
import java.io.Closeable
import java.io.InputStream
import java.io.OutputStream

/** An open byte stream to the robot's console, whatever carries it */
interface RobotLink : Closeable {
    val input: InputStream
    val output: OutputStream
}

/** Classic Bluetooth: the RFCOMM socket's own streams */
class SocketLink(private val socket: BluetoothSocket) : RobotLink {
    override val input: InputStream get() = socket.inputStream
    override val output: OutputStream get() = socket.outputStream
    override fun close() = socket.close()
}
