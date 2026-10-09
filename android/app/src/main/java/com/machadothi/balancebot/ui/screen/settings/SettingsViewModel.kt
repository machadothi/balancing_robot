package com.machadothi.balancebot.ui.screen.settings

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.model.LinkKind
import com.machadothi.balancebot.model.RobotParam
import com.machadothi.balancebot.model.RobotParams
import com.machadothi.balancebot.model.WifiNetwork
import com.machadothi.balancebot.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import javax.inject.Inject

data class ParamUi(
    val param: RobotParam,
    /** Value on the robot, null until read */
    val current: String? = null,
    /** What the user typed */
    val edit: String = "",
    val error: String? = null,
)

data class WifiUi(
    val ssid: String = "",
    val password: String = "",
    /** off, connecting or connected */
    val state: String = "?",
    val ip: String = "",
    val error: String? = null,
    /** Last AT+WIFISCAN? result */
    val networks: List<WifiNetwork> = emptyList(),
    val scanning: Boolean = false,
)

@HiltViewModel
class SettingsViewModel @Inject constructor(
    private val repository: RobotRepository,
) : ViewModel() {

    val connection = repository.connection

    private val _items = MutableStateFlow(RobotParams.all.map { ParamUi(it) })
    val items = _items.asStateFlow()

    private val _message = MutableStateFlow<String?>(null)
    val message = _message.asStateFlow()

    /** The Atom's Wi-Fi for the camera (AT+WIFI?), null over the HC-06 */
    private val _wifi = MutableStateFlow<WifiUi?>(null)
    val wifi = _wifi.asStateFlow()

    init {
        viewModelScope.launch {
            repository.connection.collect {
                _wifi.value = null
                if (it is ConnectionState.Connected) {
                    load()
                    if (it.device.kind == LinkKind.BLE) readWifi()
                }
            }
        }
    }

    fun readWifi() {
        viewModelScope.launch {
            repository.wifiStatus()
                .onSuccess { status ->
                    val parts = status.split(',')
                    val ssid = parts.getOrElse(0) { "" }
                    _wifi.update { old ->
                        (old ?: WifiUi(ssid = ssid)).copy(
                            state = parts.getOrElse(1) { "?" },
                            ip = parts.getOrElse(2) { "" },
                        )
                    }
                }
                .onFailure { e -> _wifi.update { (it ?: WifiUi()).copy(error = e.message) } }
        }
    }

    /** Ask the Atom which networks it sees; a single one in range is picked right away */
    fun scanWifi() {
        _wifi.update { (it ?: WifiUi()).copy(scanning = true, error = null) }
        viewModelScope.launch {
            repository.scanWifi()
                .onSuccess { list ->
                    _wifi.update { w ->
                        val current = w ?: WifiUi()
                        current.copy(
                            networks = list,
                            scanning = false,
                            ssid = if (current.ssid.isBlank() && list.size == 1) list[0].ssid else current.ssid,
                            error = if (list.isEmpty()) "No 2.4 GHz network found" else null,
                        )
                    }
                }
                .onFailure { e -> _wifi.update { (it ?: WifiUi()).copy(scanning = false, error = "Scan failed: ${e.message}") } }
        }
    }

    fun onWifiEdit(ssid: String, password: String) =
        _wifi.update { (it ?: WifiUi()).copy(ssid = ssid, password = password, error = null) }

    fun saveWifi() {
        val w = _wifi.value ?: return
        viewModelScope.launch {
            repository.setWifi(w.ssid.trim(), w.password)
                .onSuccess {
                    _wifi.update { it?.copy(password = "", state = "connecting", ip = "", error = null) }
                    _message.value = "Atom joins \"${w.ssid.trim()}\"; the camera appears in Drive once it is connected"
                }
                .onFailure { e -> _wifi.update { it?.copy(error = e.message) } }
        }
    }

    /** Read every value from the robot */
    fun load() {
        viewModelScope.launch {
            RobotParams.all.forEach { p ->
                repository.readParam(p)
                    .onSuccess { v -> change(p) { it.copy(current = v, edit = v, error = null) } }
                    .onFailure { e -> change(p) { it.copy(error = e.message) } }
            }
            _message.value = "Read from the robot"
        }
    }

    fun onEdit(param: RobotParam, text: String) =
        change(param) { it.copy(edit = text, error = param.validate(text)) }

    fun apply(param: RobotParam, value: String = _items.value.first { it.param == param }.edit) {
        viewModelScope.launch {
            repository.writeParam(param, value)
                .onSuccess {
                    val confirmed = repository.readParam(param).getOrNull() ?: value
                    change(param) { it.copy(current = confirmed, edit = confirmed, error = null) }
                    _message.value = "${param.label} = $confirmed"
                }
                .onFailure { e -> change(param) { it.copy(error = e.message) } }
        }
    }

    fun restoreDefaults() {
        viewModelScope.launch {
            repository.restoreDefaults()
                .onSuccess { load() }
                .onFailure { _message.value = "Defaults failed: ${it.message}" }
        }
    }

    private fun change(param: RobotParam, transform: (ParamUi) -> ParamUi) =
        _items.update { list -> list.map { if (it.param == param) transform(it) else it } }
}
