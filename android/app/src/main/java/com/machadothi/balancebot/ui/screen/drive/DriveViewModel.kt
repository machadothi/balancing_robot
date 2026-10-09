package com.machadothi.balancebot.ui.screen.drive

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.machadothi.balancebot.model.DriveCommand
import com.machadothi.balancebot.model.DriveMapping
import com.machadothi.balancebot.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class DriveViewModel @Inject constructor(
    private val repository: RobotRepository,
) : ViewModel() {

    val connection = repository.connection
    val live = repository.live

    /** The jog only sends while this is on: a pocketed phone does not drive */
    private val _driveEnabled = MutableStateFlow(false)
    val driveEnabled = _driveEnabled.asStateFlow()

    /** Speed at full stick, % of full wheel speed; small to start with */
    private val _maxSpeed = MutableStateFlow(10f)
    val maxSpeed = _maxSpeed.asStateFlow()

    private val _maxTurn = MutableStateFlow(30f)
    val maxTurn = _maxTurn.asStateFlow()

    private val _command = MutableStateFlow(DriveCommand.STOP)
    val command = _command.asStateFlow()

    private val _message = MutableStateFlow<String?>(null)
    val message = _message.asStateFlow()

    private var stick = 0f to 0f

    fun setDriveEnabled(on: Boolean) {
        _driveEnabled.value = on
        update()
    }

    fun setMaxSpeed(v: Float) {
        _maxSpeed.value = v
        update()
    }

    fun setMaxTurn(v: Float) {
        _maxTurn.value = v
        update()
    }

    fun onStick(x: Float, y: Float) {
        stick = x to y
        update()
    }

    private fun update() {
        val command = if (_driveEnabled.value) {
            DriveMapping.map(stick.first, stick.second, _maxSpeed.value, _maxTurn.value)
        } else {
            DriveCommand.STOP
        }
        _command.value = command
        repository.setDrive(command, _driveEnabled.value)
    }

    fun enable() = run("Enable") { repository.enable() }

    fun stop() {
        _driveEnabled.value = false
        stick = 0f to 0f
        update()
        run("Stop") { repository.stop() }
    }

    private fun run(what: String, action: suspend () -> Result<Unit>) {
        viewModelScope.launch {
            _message.value = action().exceptionOrNull()?.let { "$what failed: ${it.message}" }
        }
    }

    override fun onCleared() {
        repository.setDrive(DriveCommand.STOP, false)
    }
}
