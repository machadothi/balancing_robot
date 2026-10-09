package com.machadothi.balancebot.repository

import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.model.DriveCommand
import com.machadothi.balancebot.model.LiveState
import com.machadothi.balancebot.model.RobotDevice
import com.machadothi.balancebot.model.RobotParam
import com.machadothi.balancebot.model.WifiNetwork
import kotlinx.coroutines.flow.StateFlow

/** Everything the screens need from the robot, over one Bluetooth link (HC-06 or the Atom's BLE) */
interface RobotRepository {
    val connection: StateFlow<ConnectionState>

    /** Latest AT+LIVE? reply while connected, about 5 per second */
    val live: StateFlow<LiveState?>

    /** The last minute of [live], oldest first, for charts */
    val history: StateFlow<List<LiveState>>

    fun bluetoothEnabled(): Boolean
    /** Classic Bluetooth devices paired in the phone's settings (HC-06) */
    fun pairedDevices(): List<RobotDevice>

    /** Scan for the Atom bridge over BLE; [onFound] may report a device more than once */
    suspend fun scanBle(onFound: (RobotDevice) -> Unit): Result<Unit>

    suspend fun connect(device: RobotDevice)
    fun disconnect()

    /** Jog: [command] is sent repeatedly while [active], so the robot's dead-man stays fed */
    fun setDrive(command: DriveCommand, active: Boolean)

    /** AT+ENABLE */
    suspend fun enable(): Result<Unit>

    /** AT+STOP: motors off, speed and turn targets zero; also ends the jog */
    suspend fun stop(): Result<Unit>

    /** AT+DEFAULT: board default gains, setpoint and drive targets */
    suspend fun restoreDefaults(): Result<Unit>

    /** Video URL from the Atom (AT+CAM?) once its Wi-Fi is up; null over the HC-06 */
    val cameraUrl: StateFlow<String?>

    /** The Atom's Wi-Fi: "<ssid>,<off|connecting|connected>,<ip>" (AT+WIFI?) */
    suspend fun wifiStatus(): Result<String>

    /** Networks the Atom sees, strongest first (AT+WIFISCAN?, ~3 s) */
    suspend fun scanWifi(): Result<List<WifiNetwork>>

    /** AT+WIFI=<ssid>,<password>: the Atom stores them and joins that network */
    suspend fun setWifi(ssid: String, password: String): Result<Unit>

    suspend fun readParam(param: RobotParam): Result<String>
    suspend fun writeParam(param: RobotParam, value: String): Result<Unit>
}
