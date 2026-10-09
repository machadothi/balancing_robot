package com.machadothi.balancebot

import com.machadothi.balancebot.data.at.AtProtocol
import com.machadothi.balancebot.data.at.AtReply
import com.machadothi.balancebot.model.LiveState
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test

/** Replies exactly as the firmware sends them (src/cmd/at_cmd.c) */
class ProtocolTest {

    @Test
    fun cleanStripsPromptsAndCarriageReturn() {
        assertEquals("OK", AtProtocol.clean("> OK\r"))
        assertEquals("+KP:13.0000", AtProtocol.clean("> > +KP:13.0000\r"))
    }

    @Test
    fun cleanDropsBlankLinesAndTelemetry() {
        assertNull(AtProtocol.clean("\r"))
        assertNull(AtProtocol.clean("> "))
        assertNull(AtProtocol.clean("seq: 1 | t: 2 | tilt: 0.1 | drops: 0\r"))
    }

    @Test
    fun parsesFinalReplies() {
        assertEquals(AtReply.Ok, AtProtocol.parseFinal("OK"))
        assertEquals(AtReply.Value("KP", "13.0000"), AtProtocol.parseFinal("+KP:13.0000"))
        assertEquals(AtReply.Value("DEADBAND", "46,43"), AtProtocol.parseFinal("+DEADBAND:46,43"))
        val error = AtProtocol.parseFinal("ERROR:4") as AtReply.Error
        assertEquals(4, error.code)
        assertEquals("value out of range", error.meaning)
        assertEquals(null, (AtProtocol.parseFinal("ERROR:Invalid command (must start with AT)") as AtReply.Error).code)
    }

    @Test
    fun ignoresEchoAndBanner() {
        assertNull(AtProtocol.parseFinal("AT+KP?"))
        assertNull(AtProtocol.parseFinal("=== Balancing Robot v1.0 ==="))
    }

    @Test
    fun parsesLiveReply() {
        val s = LiveState.parse("1,0,-0.42,3.5,-2.61,35,1234,-1200", 7)!!
        assertEquals(true, s.enabled)
        assertEquals(false, s.balanced)
        assertEquals(-0.42f, s.tilt, 1e-6f)
        assertEquals(3.5f, s.speed, 1e-6f)
        assertEquals(-2.61f, s.setpoint, 1e-6f)
        assertEquals(35, s.output)
        assertEquals(1234L, s.encoderLeft)
        assertEquals(-1200L, s.encoderRight)
        assertEquals(7L, s.timeMs)
    }

    @Test
    fun rejectsMalformedLiveReply() {
        assertNull(LiveState.parse("1,0,-0.42", 0))
        assertNull(LiveState.parse("1,0,x,3.5,-2.61,35,1,2", 0))
    }
}
