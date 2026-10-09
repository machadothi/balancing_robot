package com.machadothi.balancebot.ui.screen.connect

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.machadothi.balancebot.model.PairedDevice
import com.machadothi.balancebot.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class ConnectViewModel @Inject constructor(
    private val repository: RobotRepository,
) : ViewModel() {

    val connection = repository.connection

    private val _devices = MutableStateFlow<List<PairedDevice>>(emptyList())
    val devices = _devices.asStateFlow()

    private val _bluetoothOn = MutableStateFlow(true)
    val bluetoothOn = _bluetoothOn.asStateFlow()

    fun refresh() {
        _bluetoothOn.value = repository.bluetoothEnabled()
        _devices.value = repository.pairedDevices()
    }

    fun connect(device: PairedDevice) {
        viewModelScope.launch { repository.connect(device) }
    }

    fun disconnect() = repository.disconnect()
}
