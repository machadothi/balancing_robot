package com.machadothi.balancebot

import com.machadothi.balancebot.model.DriveMapping
import com.machadothi.balancebot.model.ParamKind
import com.machadothi.balancebot.model.RobotParam
import com.machadothi.balancebot.model.RobotParams
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class ControlTest {

    @Test
    fun stickCentreIsStop() {
        assertTrue(DriveMapping.map(0f, 0f, 10f, 30f).isStop)
        assertTrue(DriveMapping.map(0.1f, -0.1f, 10f, 30f).isStop)   // inside the dead zone
    }

    @Test
    fun fullStickReachesTheLimits() {
        val forward = DriveMapping.map(0f, 1f, 10f, 30f)
        assertEquals(10f, forward.speed, 1e-4f)
        assertEquals(0f, forward.turn, 1e-4f)
        val backLeft = DriveMapping.map(-1f, -1f, 10f, 30f)
        assertEquals(-10f, backLeft.speed, 1e-4f)
        assertEquals(-30f, backLeft.turn, 1e-4f)
    }

    @Test
    fun smallDeflectionGivesFineControl() {
        // Squared response: half way out gives well under half speed
        assertTrue(DriveMapping.map(0f, 0.5f, 10f, 30f).speed < 3f)
    }

    @Test
    fun validationMatchesTheFirmwareRanges() {
        val kp = RobotParams.all.first { it.command == "KP" }
        assertNull(kp.validate("13"))
        assertNotNull(kp.validate("-1"))
        assertNotNull(kp.validate("abc"))

        val cap = RobotParam("OUTLIMIT", "", "", "", ParamKind.INTEGER, 20.0, 100.0)
        assertNull(cap.validate("70"))
        assertNotNull(cap.validate("70.5"))
        assertNotNull(cap.validate("10"))

        val deadband = RobotParams.all.first { it.command == "DEADBAND" }
        assertNull(deadband.validate("46,43"))
        assertNull(deadband.validate("46, 43"))
        assertNotNull(deadband.validate("46"))
        assertNotNull(deadband.validate("46,300"))

        val vloop = RobotParams.all.first { it.command == "VLOOP" }
        assertNull(vloop.validate("1"))
        assertNotNull(vloop.validate("2"))
    }
}
