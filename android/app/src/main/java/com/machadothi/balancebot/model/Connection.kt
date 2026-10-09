package com.machadothi.balancebot.model

/** How the phone reaches the robot: the module on its Bluetooth header (BT_MODULE) */
enum class LinkKind(val label: String) {
    /** HC-06, classic Bluetooth serial; paired in the phone's settings */
    CLASSIC("HC-06"),

    /** AtomS3R-CAM bridge, BLE; found by scanning, also carries the camera */
    BLE("Atom BLE"),
}

data class RobotDevice(val name: String, val address: String, val kind: LinkKind) {
    /** The robot advertises as "balancing robot" (BOARD_BT_NAME, atom/), a fresh HC-06 as "HC-06" */
    val looksLikeRobot: Boolean get() = name.startsWith("HC-") || name.contains("robot", ignoreCase = true)
}

sealed interface ConnectionState {
    data object Disconnected : ConnectionState
    data class Connecting(val device: RobotDevice) : ConnectionState
    data class Connected(val device: RobotDevice) : ConnectionState
    data class Failed(val message: String) : ConnectionState
}
