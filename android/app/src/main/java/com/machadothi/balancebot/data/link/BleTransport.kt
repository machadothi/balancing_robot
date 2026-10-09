package com.machadothi.balancebot.data.link

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothStatusCodes
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.util.Log
import com.machadothi.balancebot.model.LinkKind
import com.machadothi.balancebot.model.RobotDevice
import dagger.hilt.android.qualifiers.ApplicationContext
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withTimeoutOrNull
import java.io.ByteArrayOutputStream
import java.io.IOException
import java.io.InputStream
import java.io.InterruptedIOException
import java.io.OutputStream
import java.util.UUID
import java.util.concurrent.LinkedBlockingQueue
import javax.inject.Inject
import javax.inject.Singleton

/**
 * BLE serial to the AtomS3R-CAM bridge (atom/): the Nordic UART Service.
 *
 * No pairing: the robot is found by scanning. Callers hold BLUETOOTH_SCAN and
 * BLUETOOTH_CONNECT (Android 12+), or location (Android 6-11, to scan).
 */
@Singleton
class BleTransport @Inject constructor(
    @ApplicationContext private val context: Context,
) {
    private val adapter: BluetoothAdapter?
        get() = context.getSystemService(BluetoothManager::class.java)?.adapter

    /**
     * Scan for [durationMs] and report every robot bridge seen: the UART service
     * in its advertisement, or "robot" in its name. A device is reported again
     * when its name arrives (scan response).
     */
    @SuppressLint("MissingPermission")
    suspend fun scan(durationMs: Long = SCAN_MS, onFound: (RobotDevice) -> Unit) {
        val bt = adapter ?: throw IOException("Bluetooth is not available")
        val scanner = bt.takeIf { it.isEnabled }?.bluetoothLeScanner ?: throw IOException("Bluetooth is off")
        val failed = CompletableDeferred<Int>()
        val callback = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult) {
                val record = result.scanRecord
                val name = record?.deviceName ?: try {
                    result.device.name
                } catch (e: SecurityException) {
                    null
                }
                val uart = record?.serviceUuids?.any { it.uuid == NUS_SERVICE } == true
                if (uart || name?.contains("robot", ignoreCase = true) == true) {
                    onFound(RobotDevice(name ?: result.device.address, result.device.address, LinkKind.BLE))
                }
            }

            override fun onScanFailed(errorCode: Int) {
                failed.complete(errorCode)
            }
        }
        val settings = ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()
        scanner.startScan(null, settings, callback)
        try {
            val code = withTimeoutOrNull(durationMs) { failed.await() }
            if (code != null) throw IOException("BLE scan failed (error $code)")
        } finally {
            try {
                scanner.stopScan(callback)
            } catch (ignored: Exception) {
            }
        }
    }

    suspend fun open(address: String): RobotLink {
        val bt = adapter ?: throw IOException("Bluetooth is not available")
        if (!bt.isEnabled) throw IOException("Bluetooth is off")
        val link = BleLink(context, bt.getRemoteDevice(address))
        try {
            link.connect()
        } catch (e: Throwable) {
            link.close()
            throw if (e is SecurityException) IOException("Bluetooth permission missing", e) else e
        }
        Log.i(TAG, "connected to $address over BLE")
        return link
    }

    companion object {
        val NUS_SERVICE: UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
        val NUS_RX: UUID = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e")
        val NUS_TX: UUID = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e")
        val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        const val SCAN_MS = 6000L
        private const val TAG = "BalanceBot"
    }
}

/**
 * One GATT connection as a byte stream. Notifications from TX feed [input];
 * [output] collects a command and writes it to RX on flush(), split to the MTU.
 */
@SuppressLint("MissingPermission")
private class BleLink(
    private val context: Context,
    private val device: BluetoothDevice,
) : RobotLink {

    @Volatile private var gatt: BluetoothGatt? = null
    @Volatile private var rx: BluetoothGattCharacteristic? = null
    @Volatile private var payload = 20          // ATT MTU - 3
    @Volatile private var closed = false

    private val incoming = LinkedBlockingQueue<ByteArray>()
    private val connected = CompletableDeferred<Unit>()

    /** Android runs one GATT operation at a time; its callback completes [pending] */
    private val opMutex = Mutex()
    @Volatile private var pending: CompletableDeferred<Int>? = null

    private val callback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED && status == BluetoothGatt.GATT_SUCCESS) {
                connected.complete(Unit)
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                connected.completeExceptionally(IOException("connection failed (GATT status $status)"))
                pending?.completeExceptionally(IOException("disconnected"))
                incoming.put(END)
            }
        }

        override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) payload = mtu - 3
            pending?.complete(status)
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            pending?.complete(status)
        }

        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            pending?.complete(status)
        }

        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            pending?.complete(status)
        }

        /** Android 13+ */
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            incoming.put(value)
        }

        /** Android 12 and older */
        @Deprecated("Deprecated in Java")
        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            c.value?.let { incoming.put(it.copyOf()) }
        }
    }

    suspend fun connect() {
        gatt = device.connectGatt(context, false, callback, BluetoothDevice.TRANSPORT_LE)
        withTimeoutOrNull(CONNECT_TIMEOUT_MS) { connected.await() }
            ?: throw IOException("no answer: Atom off, out of range, or connected to another phone?")

        // Bigger packets if the phone agrees; 20-byte writes work too
        runCatching { op("MTU request", 2000) { it.requestMtu(MTU) } }

        if (op("service discovery") { it.discoverServices() } != BluetoothGatt.GATT_SUCCESS) {
            throw IOException("service discovery failed")
        }
        val service = gatt?.getService(BleTransport.NUS_SERVICE)
            ?: throw IOException("not a robot bridge (no UART service)")
        val tx = service.getCharacteristic(BleTransport.NUS_TX) ?: throw IOException("UART TX missing")
        rx = service.getCharacteristic(BleTransport.NUS_RX) ?: throw IOException("UART RX missing")

        gatt?.setCharacteristicNotification(tx, true)
        val cccd = tx.getDescriptor(BleTransport.CCCD) ?: throw IOException("UART TX cannot notify")
        val status = op("notification setup") { writeDescriptor(it, cccd, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE) }
        if (status != BluetoothGatt.GATT_SUCCESS) throw IOException("notification setup failed ($status)")
    }

    /** Start one GATT operation and wait for its callback; returns the GATT status */
    private suspend fun op(what: String, timeoutMs: Long = OP_TIMEOUT_MS, start: (BluetoothGatt) -> Boolean): Int =
        opMutex.withLock {
            val g = gatt ?: throw IOException("not connected")
            val done = CompletableDeferred<Int>()
            pending = done
            try {
                if (!start(g)) throw IOException("$what refused")
                withTimeoutOrNull(timeoutMs) { done.await() } ?: throw IOException("$what timed out")
            } finally {
                pending = null
            }
        }

    private suspend fun send(data: ByteArray) {
        val c = rx ?: throw IOException("not connected")
        var offset = 0
        while (offset < data.size) {
            val end = minOf(data.size, offset + payload)
            val part = data.copyOfRange(offset, end)
            val status = op("write") { writeCharacteristic(it, c, part) }
            if (status != BluetoothGatt.GATT_SUCCESS) throw IOException("write failed ($status)")
            offset = end
        }
    }

    override val input: InputStream = object : InputStream() {
        private var chunk = ByteArray(0)
        private var pos = 0

        override fun read(): Int {
            val one = ByteArray(1)
            return if (read(one, 0, 1) < 0) -1 else one[0].toInt() and 0xFF
        }

        override fun read(b: ByteArray, off: Int, len: Int): Int {
            if (len == 0) return 0
            while (pos >= chunk.size) {
                val next = try {
                    incoming.take()
                } catch (e: InterruptedException) {
                    throw InterruptedIOException()
                }
                if (next === END) {
                    incoming.put(END)       // every later read ends too
                    return -1
                }
                chunk = next
                pos = 0
            }
            val n = minOf(len, chunk.size - pos)
            System.arraycopy(chunk, pos, b, off, n)
            pos += n
            return n
        }
    }

    override val output: OutputStream = object : OutputStream() {
        private val buffer = ByteArrayOutputStream()

        @Synchronized override fun write(b: Int) = buffer.write(b)

        @Synchronized override fun write(b: ByteArray, off: Int, len: Int) = buffer.write(b, off, len)

        override fun flush() {
            val data = synchronized(this) { buffer.toByteArray().also { buffer.reset() } }
            if (closed) throw IOException("link closed")
            if (data.isNotEmpty()) runBlocking { send(data) }
        }
    }

    override fun close() {
        if (closed) return
        closed = true
        incoming.put(END)
        pending?.completeExceptionally(IOException("closed"))
        gatt?.let {
            try {
                it.disconnect()
                it.close()
            } catch (ignored: Exception) {
            }
        }
        gatt = null
    }

    @Suppress("DEPRECATION")
    private fun writeDescriptor(g: BluetoothGatt, d: BluetoothGattDescriptor, value: ByteArray): Boolean =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            g.writeDescriptor(d, value) == BluetoothStatusCodes.SUCCESS
        } else {
            d.value = value
            g.writeDescriptor(d)
        }

    @Suppress("DEPRECATION")
    private fun writeCharacteristic(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray): Boolean {
        val type = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            g.writeCharacteristic(c, value, type) == BluetoothStatusCodes.SUCCESS
        } else {
            c.writeType = type
            c.value = value
            g.writeCharacteristic(c)
        }
    }

    private companion object {
        val END = ByteArray(0)              // end-of-stream marker in [incoming]
        const val MTU = 247
        const val CONNECT_TIMEOUT_MS = 10_000L
        const val OP_TIMEOUT_MS = 5_000L
    }
}
