package com.machadothi.balancebot.model

enum class ParamKind { NUMBER, INTEGER, SWITCH, PAIR }

/**
 * A tunable value of the robot, read with AT+<command>? and written with
 * AT+<command>=<value>. Ranges match the firmware's command table, so the app
 * rejects what the robot would answer with ERROR:4.
 */
data class RobotParam(
    val command: String,
    val label: String,
    val help: String,
    val group: String,
    val kind: ParamKind,
    val min: Double = 0.0,
    val max: Double = 0.0,
) {
    /** Error text for [input], or null if the robot will accept it */
    fun validate(input: String): String? {
        val text = input.trim()
        return when (kind) {
            ParamKind.SWITCH -> if (text == "0" || text == "1") null else "0 or 1"
            ParamKind.NUMBER -> {
                val v = text.toDoubleOrNull()
                if (v == null) "not a number" else inRange(v)
            }
            ParamKind.INTEGER -> {
                val v = text.toIntOrNull()
                if (v == null) "not a whole number" else inRange(v.toDouble())
            }
            ParamKind.PAIR -> {
                val parts = text.split(',').map { it.trim().toIntOrNull() }
                if (parts.size != 2 || parts.any { it == null }) "two whole numbers: left,right"
                else parts.firstNotNullOfOrNull { inRange(it!!.toDouble()) }
            }
        }
    }

    private fun inRange(v: Double): String? =
        if (v < min || v > max) "between ${fmt(min)} and ${fmt(max)}" else null

    private fun fmt(v: Double) = if (v == Math.floor(v)) v.toLong().toString() else v.toString()
}

object RobotParams {
    const val BALANCE = "Balance"
    const val SPEED = "Speed loop"
    const val HARDWARE = "Sensor and motors"

    val all = listOf(
        RobotParam("KP", "KP", "Push per degree of tilt", BALANCE, ParamKind.NUMBER, 0.0, 100.0),
        RobotParam("KI", "KI", "Integral of the tilt; keep 0 while the speed loop is on", BALANCE, ParamKind.NUMBER, 0.0, 10.0),
        RobotParam("KD", "KD", "Damping, per deg/s", BALANCE, ParamKind.NUMBER, 0.0, 10.0),
        RobotParam("DGYRO", "D from the gyro", "D term from the gyro rate (1) or the angle difference (0)", BALANCE, ParamKind.SWITCH),
        RobotParam("SETPOINT", "Balance point", "Tilt the robot balances at, degrees", BALANCE, ParamKind.NUMBER, -10.0, 10.0),
        RobotParam("OUTLIMIT", "Output cap", "Motor power limit, percent", BALANCE, ParamKind.INTEGER, 20.0, 100.0),
        RobotParam("VLOOP", "Speed loop", "Keeps the robot in place and follows the jog speed", SPEED, ParamKind.SWITCH),
        RobotParam("VKP", "VKP", "Lean per % of speed error, degrees", SPEED, ParamKind.NUMBER, 0.0, 1.0),
        RobotParam("VKI", "VKI", "Integral of the speed error", SPEED, ParamKind.NUMBER, 0.0, 1.0),
        RobotParam("ALPHA", "Filter gyro weight", "Complementary filter: higher trusts the gyro more", HARDWARE, ParamKind.NUMBER, 0.9, 0.999),
        RobotParam("DEADBAND", "Motor dead zone", "Smallest command that turns each wheel: left,right", HARDWARE, ParamKind.PAIR, 0.0, 200.0),
    )
}
