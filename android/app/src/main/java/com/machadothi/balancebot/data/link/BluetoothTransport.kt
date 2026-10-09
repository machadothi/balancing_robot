package com.machadothi.balancebot.data.link

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.Context
import android.util.Log
import com.machadothi.balancebot.model.PairedDevice
import dagger.hilt.android.qualifiers.ApplicationContext
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.withContext
import java.io.IOException
import java.util.UUID
import javax.inject.Inject
import javax.inject.Singleton

/**
 * Classic Bluetooth serial (SPP) to the robot's HC-06 module.
 *
 * Pairing (PIN 1234) happens in the phone's Bluetooth settings; the app lists
 * paired devices only. Callers hold BLUETOOTH_CONNECT (Android 12+).
 */
@Singleton
class BluetoothTransport @Inject constructor(
    @ApplicationContext private val context: Context,
) {
    private val adapter: BluetoothAdapter?
        get() = context.getSystemService(BluetoothManager::class.java)?.adapter

    fun isEnabled(): Boolean = adapter?.isEnabled == true

    @SuppressLint("MissingPermission")
    fun pairedDevices(): List<PairedDevice> = try {
        adapter?.bondedDevices.orEmpty()
            .map { PairedDevice(it.name ?: it.address, it.address) }
            .sortedWith(compareByDescending<PairedDevice> { it.looksLikeRobot }.thenBy { it.name })
    } catch (e: SecurityException) {
        emptyList()
    }

    /**
     * Connect to [address]. HC-0x modules differ in what they accept, so try the
     * secure SPP socket, then the insecure one, then RFCOMM channel 1 directly.
     * No discovery is involved (cancelDiscovery() would need BLUETOOTH_SCAN).
     */
    @SuppressLint("MissingPermission")
    suspend fun open(address: String): BluetoothSocket = withContext(Dispatchers.IO) {
        val bt = adapter ?: throw IOException("Bluetooth is not available")
        if (!bt.isEnabled) throw IOException("Bluetooth is off")
        val device = bt.getRemoteDevice(address)

        val attempts: List<Pair<String, () -> BluetoothSocket>> = listOf(
            "secure SPP" to { device.createRfcommSocketToServiceRecord(SPP_UUID) },
            "insecure SPP" to { device.createInsecureRfcommSocketToServiceRecord(SPP_UUID) },
            "RFCOMM channel 1" to {
                device.javaClass.getMethod("createRfcommSocket", Int::class.javaPrimitiveType)
                    .invoke(device, 1) as BluetoothSocket
            },
        )
        var lastError: Exception? = null
        for ((name, create) in attempts) {
            var socket: BluetoothSocket? = null
            try {
                socket = create()
                socket.connect()
                Log.i(TAG, "connected to $address with $name")
                return@withContext socket
            } catch (e: Exception) {
                Log.w(TAG, "$name to $address failed: ${e.message}", e)
                lastError = e
                try {
                    socket?.close()
                } catch (ignored: IOException) {
                }
            }
        }
        val reason = when (lastError) {
            is SecurityException -> "Bluetooth permission missing"
            else -> "robot off, out of range, or connected to another device?"
        }
        throw IOException("could not connect: $reason (${lastError?.message})", lastError)
    }

    companion object {
        /** Serial Port Profile */
        val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        private const val TAG = "BalanceBot"
    }
}
