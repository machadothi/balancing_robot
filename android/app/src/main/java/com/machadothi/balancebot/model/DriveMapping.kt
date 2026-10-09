package com.machadothi.balancebot.model

import kotlin.math.abs
import kotlin.math.sign

/** What the jog sends: AT+VELOCITY (speed %) and AT+TURN */
data class DriveCommand(val speed: Float, val turn: Float) {
    val isStop: Boolean get() = speed == 0f && turn == 0f

    companion object {
        val STOP = DriveCommand(0f, 0f)
    }
}

object DriveMapping {
    /** Stick deflection ignored around the centre, so a resting thumb is a stop */
    const val STICK_DEAD_ZONE = 0.12f

    /**
     * Joystick position to drive targets.
     *
     * @param x  -1 (left) .. 1 (right)
     * @param y  -1 (back) .. 1 (forward)
     * @param maxSpeed  speed at full forward deflection, % (firmware range +-100)
     * @param maxTurn   turn at full sideways deflection (firmware range +-100)
     */
    fun map(x: Float, y: Float, maxSpeed: Float, maxTurn: Float): DriveCommand =
        DriveCommand(speed = shape(y) * maxSpeed, turn = shape(x) * maxTurn)

    /** Dead zone, then rescaled so the output still reaches 1, squared for fine control near 0 */
    private fun shape(v: Float): Float {
        val clamped = v.coerceIn(-1f, 1f)
        if (abs(clamped) <= STICK_DEAD_ZONE) return 0f
        val scaled = (abs(clamped) - STICK_DEAD_ZONE) / (1f - STICK_DEAD_ZONE)
        return sign(clamped) * scaled * scaled
    }
}
