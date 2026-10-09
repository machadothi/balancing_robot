package com.machadothi.balancebot.repository

import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.model.DriveCommand
import com.machadothi.balancebot.model.LiveState
import com.machadothi.balancebot.model.PairedDevice
import com.machadothi.balancebot.model.RobotParam
import kotlinx.coroutines.flow.StateFlow

/** Everything the screens need from the robot, over one Bluetooth link */
interface RobotRepository {
    val connection: StateFlow<ConnectionState>

    /** Latest AT+LIVE? reply while connected, about 5 per second */
    val live: StateFlow<LiveState?>

    /** The last minute of [live], oldest first, for charts */
    val history: StateFlow<List<LiveState>>

    fun bluetoothEnabled(): Boolean
    fun pairedDevices(): List<PairedDevice>

    suspend fun connect(device: PairedDevice)
    fun disconnect()

    /** Jog: [command] is sent repeatedly while [active], so the robot's dead-man stays fed */
    fun setDrive(command: DriveCommand, active: Boolean)

    /** AT+ENABLE */
    suspend fun enable(): Result<Unit>

    /** AT+STOP: motors off, speed and turn targets zero; also ends the jog */
    suspend fun stop(): Result<Unit>

    /** AT+DEFAULT: board default gains, setpoint and drive targets */
    suspend fun restoreDefaults(): Result<Unit>

    suspend fun readParam(param: RobotParam): Result<String>
    suspend fun writeParam(param: RobotParam, value: String): Result<Unit>
}
