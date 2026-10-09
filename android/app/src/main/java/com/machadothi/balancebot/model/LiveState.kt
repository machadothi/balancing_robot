package com.machadothi.balancebot.model

/** One AT+LIVE? reply (src/robot/robot_commands.c) */
data class LiveState(
    val enabled: Boolean,
    val balanced: Boolean,
    /** Tilt, degrees, 0 = upright, positive = leaning forward */
    val tilt: Float,
    /** Forward speed, % of full wheel speed */
    val speed: Float,
    /** Balance target the controller uses after the speed loop, degrees */
    val setpoint: Float,
    /** Balance PID output, motor counts (+-255) */
    val output: Int,
    val encoderLeft: Long,
    val encoderRight: Long,
    /** Phone time of the reply, ms */
    val timeMs: Long,
) {
    companion object {
        /** "enabled,balanced,tilt,speed,setpoint,output,encL,encR"; null if malformed */
        fun parse(value: String, timeMs: Long): LiveState? {
            val f = value.split(',').map { it.trim() }
            if (f.size != 8) return null
            return try {
                LiveState(
                    enabled = f[0] == "1",
                    balanced = f[1] == "1",
                    tilt = f[2].toFloat(),
                    speed = f[3].toFloat(),
                    setpoint = f[4].toFloat(),
                    output = f[5].toInt(),
                    encoderLeft = f[6].toLong(),
                    encoderRight = f[7].toLong(),
                    timeMs = timeMs,
                )
            } catch (e: NumberFormatException) {
                null
            }
        }
    }
}
