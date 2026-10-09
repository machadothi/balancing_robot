package com.machadothi.balancebot.ui.screen.settings

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.model.RobotParam
import com.machadothi.balancebot.model.RobotParams
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

@HiltViewModel
class SettingsViewModel @Inject constructor(
    private val repository: RobotRepository,
) : ViewModel() {

    val connection = repository.connection

    private val _items = MutableStateFlow(RobotParams.all.map { ParamUi(it) })
    val items = _items.asStateFlow()

    private val _message = MutableStateFlow<String?>(null)
    val message = _message.asStateFlow()

    init {
        viewModelScope.launch {
            repository.connection.collect { if (it is ConnectionState.Connected) load() }
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
