package com.machadothi.balancebot.ui.screen.connect

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.machadothi.balancebot.model.RobotDevice
import com.machadothi.balancebot.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class ConnectViewModel @Inject constructor(
    private val repository: RobotRepository,
) : ViewModel() {

    val connection = repository.connection

    /** HC-06 and other classic devices paired in the phone's settings */
    private val _paired = MutableStateFlow<List<RobotDevice>>(emptyList())
    val paired = _paired.asStateFlow()

    /** Atom bridges found by the last BLE scan */
    private val _nearby = MutableStateFlow<List<RobotDevice>>(emptyList())
    val nearby = _nearby.asStateFlow()

    private val _scanning = MutableStateFlow(false)
    val scanning = _scanning.asStateFlow()

    private val _scanError = MutableStateFlow<String?>(null)
    val scanError = _scanError.asStateFlow()

    private val _bluetoothOn = MutableStateFlow(true)
    val bluetoothOn = _bluetoothOn.asStateFlow()

    private var scanJob: Job? = null

    fun refresh() {
        _bluetoothOn.value = repository.bluetoothEnabled()
        _paired.value = repository.pairedDevices()
    }

    fun scan() {
        if (_scanning.value || !repository.bluetoothEnabled()) return
        scanJob = viewModelScope.launch {
            _scanning.value = true
            _scanError.value = null
            _nearby.value = emptyList()
            repository.scanBle { found ->
                _nearby.update { list ->
                    val old = list.firstOrNull { it.address == found.address }
                    when {
                        old == null -> list + found
                        // The name comes with the scan response, after the first sighting
                        old.name == old.address && found.name != found.address ->
                            list.map { if (it.address == found.address) found else it }
                        else -> list
                    }
                }
            }.onFailure { _scanError.value = it.message }
            _scanning.value = false
        }
    }

    fun connect(device: RobotDevice) {
        scanJob?.cancel()       // a running scan slows the connection down
        _scanning.value = false
        viewModelScope.launch { repository.connect(device) }
    }

    fun disconnect() = repository.disconnect()
}
