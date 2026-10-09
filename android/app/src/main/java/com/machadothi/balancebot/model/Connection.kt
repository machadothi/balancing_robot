package com.machadothi.balancebot.model

data class PairedDevice(val name: String, val address: String) {
    /** The robot's module advertises as "balancing robot" (BOARD_BT_NAME), a fresh one as "HC-06" */
    val looksLikeRobot: Boolean get() = name.startsWith("HC-") || name.contains("robot", ignoreCase = true)
}

sealed interface ConnectionState {
    data object Disconnected : ConnectionState
    data class Connecting(val device: PairedDevice) : ConnectionState
    data class Connected(val device: PairedDevice) : ConnectionState
    data class Failed(val message: String) : ConnectionState
}
