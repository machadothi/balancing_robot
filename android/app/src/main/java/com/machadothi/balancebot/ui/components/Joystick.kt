package com.machadothi.balancebot.ui.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import kotlin.math.min

/**
 * Thumb stick. Reports (x, y) in -1..1 with y = +1 at the top (forward);
 * returns to the centre, and reports (0, 0), when released.
 */
@Composable
fun Joystick(
    modifier: Modifier = Modifier,
    enabled: Boolean = true,
    baseColor: Color,
    knobColor: Color,
    onMove: (x: Float, y: Float) -> Unit,
) {
    var knob by remember { mutableStateOf(Offset.Zero) }   // from the centre, px

    Canvas(
        modifier = modifier
            .aspectRatio(1f)
            .pointerInput(enabled) {
                if (!enabled) return@pointerInput
                val radius = min(size.width, size.height) / 2f * 0.75f
                fun report() = onMove(knob.x / radius, -knob.y / radius)
                detectDragGestures(
                    onDragStart = { start ->
                        knob = clamp(start - Offset(size.width / 2f, size.height / 2f), radius)
                        report()
                    },
                    onDragEnd = { knob = Offset.Zero; onMove(0f, 0f) },
                    onDragCancel = { knob = Offset.Zero; onMove(0f, 0f) },
                ) { change, drag ->
                    change.consume()
                    knob = clamp(knob + drag, radius)
                    report()
                }
            },
    ) {
        val radius = min(size.width, size.height) / 2f * 0.75f
        val centre = Offset(size.width / 2f, size.height / 2f)
        drawCircle(baseColor.copy(alpha = if (enabled) 0.25f else 0.1f), radius = radius, center = centre)
        drawCircle(baseColor, radius = radius, center = centre, style = Stroke(width = 4f))
        drawLine(baseColor.copy(alpha = 0.4f), centre - Offset(radius, 0f), centre + Offset(radius, 0f), 2f)
        drawLine(baseColor.copy(alpha = 0.4f), centre - Offset(0f, radius), centre + Offset(0f, radius), 2f)
        drawCircle(
            knobColor.copy(alpha = if (enabled) 1f else 0.3f),
            radius = radius * 0.3f,
            center = centre + knob,
        )
    }
}

private fun clamp(offset: Offset, radius: Float): Offset {
    val distance = offset.getDistance()
    return if (distance <= radius || distance == 0f) offset else offset * (radius / distance)
}
