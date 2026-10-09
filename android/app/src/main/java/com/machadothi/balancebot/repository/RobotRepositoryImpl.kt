package com.machadothi.balancebot.repository

import android.util.Log
import com.machadothi.balancebot.data.at.AtReply
import com.machadothi.balancebot.data.at.AtSession
import com.machadothi.balancebot.data.link.BleTransport
import com.machadothi.balancebot.data.link.BluetoothTransport
import com.machadothi.balancebot.data.link.RobotLink
import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.model.DriveCommand
import com.machadothi.balancebot.model.LinkKind
import com.machadothi.balancebot.model.LiveState
import com.machadothi.balancebot.model.RobotDevice
import com.machadothi.balancebot.model.RobotParam
import com.machadothi.balancebot.model.WifiNetwork
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.io.IOException
import java.util.Locale
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
class RobotRepositoryImpl @Inject constructor(
    private val classic: BluetoothTransport,
    private val ble: BleTransport,
) : RobotRepository {

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    private var link: RobotLink? = null
    private var session: AtSession? = null
    private var pollJob: Job? = null

    private val _connection = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    override val connection: StateFlow<ConnectionState> = _connection.asStateFlow()

    private val _live = MutableStateFlow<LiveState?>(null)
    override val live: StateFlow<LiveState?> = _live.asStateFlow()

    private val _history = MutableStateFlow<List<LiveState>>(emptyList())
    override val history: StateFlow<List<LiveState>> = _history.asStateFlow()

    private val _cameraUrl = MutableStateFlow<String?>(null)
    override val cameraUrl: StateFlow<String?> = _cameraUrl.asStateFlow()

    @Volatile private var driveCommand = DriveCommand.STOP
    @Volatile private var driveActive = false

    override fun bluetoothEnabled(): Boolean = classic.isEnabled()

    override fun pairedDevices(): List<RobotDevice> = classic.pairedDevices()

    override suspend fun scanBle(onFound: (RobotDevice) -> Unit): Result<Unit> = try {
        ble.scan(onFound = onFound)
        Result.success(Unit)
    } catch (e: CancellationException) {
        throw e
    } catch (e: Exception) {
        Result.failure(if (e is SecurityException) IOException("Bluetooth scan permission missing") else e)
    }

    override suspend fun connect(device: RobotDevice) {
        disconnect()
        _connection.value = ConnectionState.Connecting(device)
        try {
            val l = when (device.kind) {
                LinkKind.CLASSIC -> classic.open(device.address)
                LinkKind.BLE -> ble.open(device.address)
            }
            link = l
            val at = AtSession(l.input, l.output, scope)
            session = at
            // A plain AT proves that the other end is the robot's console
            if (at.transact("AT", 3000) !is AtReply.Ok) throw IOException("no answer from the robot")
            _history.value = emptyList()
            _connection.value = ConnectionState.Connected(device)
            pollJob = scope.launch { pollLoop(at, camera = device.kind == LinkKind.BLE) }
        } catch (e: CancellationException) {
            close()
            _connection.value = ConnectionState.Disconnected
            throw e
        } catch (e: Exception) {
            Log.w("BalanceBot", "connect to ${device.address} failed", e)
            close()
            _connection.value = ConnectionState.Failed(e.message ?: e.javaClass.simpleName)
        }
    }

    override fun disconnect() {
        close()
        _connection.value = ConnectionState.Disconnected
    }

    private fun close() {
        driveActive = false
        driveCommand = DriveCommand.STOP
        pollJob?.cancel()
        pollJob = null
        session = null
        try {
            link?.close()
        } catch (ignored: IOException) {
        }
        link = null
        _live.value = null
        _cameraUrl.value = null
    }

    override fun setDrive(command: DriveCommand, active: Boolean) {
        driveCommand = command
        driveActive = active
    }

    /**
     * Poll AT+LIVE? and keep the jog alive. The robot zeroes speed and turn after
     * 1 s without a drive command, so a lost link stops it by itself. Over the
     * Atom, also ask for the camera URL now and then: Wi-Fi may come up later.
     */
    private suspend fun pollLoop(at: AtSession, camera: Boolean) {
        var lastCameraMs = 0L
        var lastDriveMs = 0L
        var stopSent = true
        var misses = 0
        try {
            while (scope.isActive) {
                val now = System.currentTimeMillis()
                val active = driveActive
                val command = driveCommand
                if (active && now - lastDriveMs >= DRIVE_PERIOD_MS) {
                    sendDrive(at, command)
                    lastDriveMs = now
                    stopSent = command.isStop
                } else if (!active && !stopSent) {
                    sendDrive(at, DriveCommand.STOP)
                    stopSent = true
                }

                if (camera && now - lastCameraMs >= CAMERA_CHECK_MS) {
                    lastCameraMs = now
                    val reply = at.transact("AT+CAM?")
                    if (reply is AtReply.Value && reply.name == "CAM") _cameraUrl.value = reply.value.ifEmpty { null }
                }

                when (val reply = at.transact("AT+LIVE?")) {
                    is AtReply.Value -> {
                        misses = 0
                        LiveState.parse(reply.value, now)?.let(::publish)
                    }
                    else -> if (++misses >= MAX_MISSES) throw IOException("the robot stopped answering")
                }
                delay(POLL_PERIOD_MS)
            }
        } catch (e: CancellationException) {
            throw e
        } catch (e: IOException) {
            close()
            _connection.value = ConnectionState.Failed("link lost: ${e.message}")
        }
    }

    private suspend fun sendDrive(at: AtSession, command: DriveCommand) {
        at.transact("AT+VELOCITY=" + format(command.speed))
        at.transact("AT+TURN=" + format(command.turn))
    }

    private fun publish(state: LiveState) {
        _live.value = state
        val cutoff = state.timeMs - HISTORY_MS
        _history.value = (_history.value + state).dropWhile { it.timeMs < cutoff }
    }

    /** One command from a screen; a link that breaks meanwhile is a failure, not a crash */
    private suspend fun ask(text: String, timeoutMs: Long = AtSession.DEFAULT_TIMEOUT_MS): Result<AtReply> {
        val at = session ?: return Result.failure(IOException("not connected"))
        return try {
            Result.success(at.transact(text, timeoutMs))
        } catch (e: IOException) {
            Result.failure(e)
        }
    }

    private suspend fun command(text: String): Result<Unit> = ask(text).mapCatching { reply ->
        when (reply) {
            AtReply.Ok, is AtReply.Value -> Unit
            is AtReply.Error -> throw IOException(reply.meaning)
            AtReply.Timeout -> throw IOException("no reply")
        }
    }

    private suspend fun query(text: String, timeoutMs: Long = AtSession.DEFAULT_TIMEOUT_MS): Result<String> =
        ask(text, timeoutMs).mapCatching { reply ->
        when (reply) {
            is AtReply.Value -> reply.value
            is AtReply.Error -> throw IOException(reply.meaning)
            else -> throw IOException("no reply")
        }
    }

    override suspend fun enable(): Result<Unit> = command("AT+ENABLE")

    override suspend fun stop(): Result<Unit> {
        driveActive = false
        driveCommand = DriveCommand.STOP
        return command("AT+STOP")
    }

    override suspend fun restoreDefaults(): Result<Unit> = command("AT+DEFAULT")

    override suspend fun readParam(param: RobotParam): Result<String> = query("AT+${param.command}?")

    override suspend fun wifiStatus(): Result<String> = query("AT+WIFI?")

    override suspend fun scanWifi(): Result<List<WifiNetwork>> =
        query("AT+WIFISCAN?", WIFI_SCAN_TIMEOUT_MS).map(WifiNetwork::parseList)

    override suspend fun setWifi(ssid: String, password: String): Result<Unit> {
        if (ssid.isEmpty() || ssid.contains(',')) return Result.failure(IllegalArgumentException("SSID empty or with a comma"))
        if (password.isNotEmpty() && password.length < 8) return Result.failure(IllegalArgumentException("WPA passwords have 8+ characters"))
        return command("AT+WIFI=$ssid,$password").onSuccess { _cameraUrl.value = null }
    }

    override suspend fun writeParam(param: RobotParam, value: String): Result<Unit> {
        param.validate(value)?.let { return Result.failure(IllegalArgumentException(it)) }
        return command("AT+${param.command}=${value.trim().replace(" ", "")}")
    }

    private fun format(v: Float) = String.format(Locale.US, "%.1f", v.coerceIn(-100f, 100f))

    private companion object {
        const val POLL_PERIOD_MS = 150L       // ~5 replies/s at 9600 baud with the jog
        const val DRIVE_PERIOD_MS = 300L      // well inside the robot's 1 s dead-man
        const val HISTORY_MS = 60_000L
        const val MAX_MISSES = 5
        const val CAMERA_CHECK_MS = 5_000L
        const val WIFI_SCAN_TIMEOUT_MS = 10_000L
    }
}
