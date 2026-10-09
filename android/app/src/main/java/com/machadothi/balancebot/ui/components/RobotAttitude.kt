package com.machadothi.balancebot.ui.components

import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.CornerRadius
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.PathEffect
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import com.machadothi.balancebot.model.LiveState
import com.machadothi.balancebot.ui.theme.StateColors
import kotlin.math.abs

/**
 * The robot drawn as it stands: tilted by the measured angle (forward = to the
 * right), green while balancing, amber when tilted, grey when the motors are
 * off. The dashed line is the balance target the controller aims for.
 */
@Composable
fun RobotAttitude(live: LiveState?, modifier: Modifier = Modifier) {
    val tilt by animateFloatAsState(live?.tilt?.coerceIn(-90f, 90f) ?: 0f, tween(120), label = "tilt")
    val target by animateFloatAsState(live?.setpoint ?: 0f, tween(300), label = "target")
    val color = when {
        live == null || !live.enabled -> StateColors.off
        abs(live.tilt) < 5f -> StateColors.balancing
        else -> StateColors.tilted
    }
    val ground = MaterialTheme.colorScheme.outline
    val dark = MaterialTheme.colorScheme.surface

    Card(modifier = modifier) {
        Box(Modifier.fillMaxWidth().height(190.dp)) {
            Canvas(Modifier.fillMaxWidth().height(190.dp)) {
                val unit = size.height / 10f
                val axle = Offset(size.width / 2f, size.height - unit * 1.6f)
                drawLine(ground, Offset(unit, axle.y + unit * 1.1f), Offset(size.width - unit, axle.y + unit * 1.1f), 4f)

                // Balance target
                rotate(target, pivot = axle) {
                    drawLine(
                        color.copy(alpha = 0.5f), axle, axle - Offset(0f, unit * 7.5f), 3f,
                        pathEffect = PathEffect.dashPathEffect(floatArrayOf(14f, 10f)),
                    )
                }
                // The robot, leaning about its axle
                rotate(tilt, pivot = axle) {
                    val bodyW = unit * 2.6f
                    val bodyH = unit * 4.2f
                    val bodyTop = axle.y - unit * 1.1f - bodyH
                    drawLine(color, Offset(axle.x, bodyTop), Offset(axle.x, bodyTop - unit * 0.9f), 6f)
                    drawCircle(StateColors.tilted, unit * 0.35f, Offset(axle.x, bodyTop - unit * 1.0f))
                    drawRoundRect(color, Offset(axle.x - bodyW / 2, bodyTop), Size(bodyW, bodyH), CornerRadius(unit * 0.7f))
                    drawRoundRect(dark, Offset(axle.x - bodyW * 0.36f, bodyTop + unit * 0.6f),
                        Size(bodyW * 0.72f, unit * 1.3f), CornerRadius(unit * 0.4f))
                    drawCircle(color, unit * 0.25f, Offset(axle.x - bodyW * 0.17f, bodyTop + unit * 1.25f))
                    drawCircle(color, unit * 0.25f, Offset(axle.x + bodyW * 0.17f, bodyTop + unit * 1.25f))
                    drawCircle(color, unit * 1.2f, axle)
                    drawCircle(dark, unit * 0.8f, axle)
                    drawCircle(color, unit * 0.3f, axle)
                }
            }
            Column(Modifier.align(Alignment.TopStart).padding(12.dp)) {
                Text(
                    live?.let { "%+.1f°".format(it.tilt) } ?: "–",
                    style = MaterialTheme.typography.headlineMedium,
                    fontWeight = FontWeight.Bold,
                    color = color,
                )
                Text(
                    when {
                        live == null -> "no data"
                        !live.enabled -> "motors off"
                        abs(live.tilt) < 5f -> "balancing"
                        else -> "tilted"
                    },
                    style = MaterialTheme.typography.labelMedium,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
            if (live != null) {
                Text(
                    "target %+.1f°".format(live.setpoint),
                    modifier = Modifier.align(Alignment.TopEnd).padding(12.dp),
                    style = MaterialTheme.typography.labelMedium,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                )
            }
        }
    }
}
