package com.machadothi.balancebot.repository

import android.bluetooth.BluetoothSocket
import android.util.Log
import com.machadothi.balancebot.data.at.AtReply
import com.machadothi.balancebot.data.at.AtSession
import com.machadothi.balancebot.data.link.BluetoothTransport
import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.model.DriveCommand
import com.machadothi.balancebot.model.LiveState
import com.machadothi.balancebot.model.PairedDevice
import com.machadothi.balancebot.model.RobotParam
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
    private val transport: BluetoothTransport,
) : RobotRepository {

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    private var socket: BluetoothSocket? = null
    private var session: AtSession? = null
    private var pollJob: Job? = null

    private val _connection = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    override val connection: StateFlow<ConnectionState> = _connection.asStateFlow()

    private val _live = MutableStateFlow<LiveState?>(null)
    override val live: StateFlow<LiveState?> = _live.asStateFlow()

    private val _history = MutableStateFlow<List<LiveState>>(emptyList())
    override val history: StateFlow<List<LiveState>> = _history.asStateFlow()

    @Volatile private var driveCommand = DriveCommand.STOP
    @Volatile private var driveActive = false

    override fun bluetoothEnabled(): Boolean = transport.isEnabled()

    override fun pairedDevices(): List<PairedDevice> = transport.pairedDevices()

    override suspend fun connect(device: PairedDevice) {
        disconnect()
        _connection.value = ConnectionState.Connecting(device)
        try {
            val s = transport.open(device.address)
            socket = s
            val at = AtSession(s.inputStream, s.outputStream, scope)
            session = at
            // A plain AT proves that the other end is the robot's console
            if (at.transact("AT", 3000) !is AtReply.Ok) throw IOException("no answer from the robot")
            _history.value = emptyList()
            _connection.value = ConnectionState.Connected(device)
            pollJob = scope.launch { pollLoop(at) }
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
            socket?.close()
        } catch (ignored: IOException) {
        }
        socket = null
        _live.value = null
    }

    override fun setDrive(command: DriveCommand, active: Boolean) {
        driveCommand = command
        driveActive = active
    }

    /**
     * Poll AT+LIVE? and keep the jog alive. The robot zeroes speed and turn after
     * 1 s without a drive command, so a lost link stops it by itself.
     */
    private suspend fun pollLoop(at: AtSession) {
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

    private suspend fun command(text: String): Result<Unit> {
        val at = session ?: return Result.failure(IOException("not connected"))
        return when (val reply = at.transact(text)) {
            AtReply.Ok -> Result.success(Unit)
            is AtReply.Error -> Result.failure(IOException(reply.meaning))
            AtReply.Timeout -> Result.failure(IOException("no reply"))
            is AtReply.Value -> Result.success(Unit)
        }
    }

    override suspend fun enable(): Result<Unit> = command("AT+ENABLE")

    override suspend fun stop(): Result<Unit> {
        driveActive = false
        driveCommand = DriveCommand.STOP
        return command("AT+STOP")
    }

    override suspend fun restoreDefaults(): Result<Unit> = command("AT+DEFAULT")

    override suspend fun readParam(param: RobotParam): Result<String> {
        val at = session ?: return Result.failure(IOException("not connected"))
        return when (val reply = at.transact("AT+${param.command}?")) {
            is AtReply.Value -> Result.success(reply.value)
            is AtReply.Error -> Result.failure(IOException(reply.meaning))
            else -> Result.failure(IOException("no reply"))
        }
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
    }
}
