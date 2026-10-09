package com.machadothi.balancebot.ui.components

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.width
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.unit.dp
import kotlin.math.abs
import kotlin.math.max

data class ChartSeries(val label: String, val color: Color, val values: List<Float>)

/**
 * Lines over time, symmetric around 0; the range follows the largest value,
 * at least [minRange], so a calm robot does not look noisy.
 */
@Composable
fun LineChart(series: List<ChartSeries>, title: String, minRange: Float, modifier: Modifier = Modifier) {
    val axis = MaterialTheme.colorScheme.outline
    val range = max(minRange, series.flatMap { it.values }.maxOfOrNull { abs(it) } ?: 0f) * 1.1f

    Column(modifier = modifier) {
        Row {
            Text(title, style = MaterialTheme.typography.labelLarge)
            series.forEach {
                Spacer(Modifier.width(12.dp))
                Text(it.label, color = it.color, style = MaterialTheme.typography.labelLarge)
            }
        }
        Text("±%.1f".format(range), style = MaterialTheme.typography.bodySmall)
        Canvas(modifier = Modifier.fillMaxWidth().height(160.dp)) {
            val mid = size.height / 2f
            drawLine(axis, Offset(0f, mid), Offset(size.width, mid), 1f)
            series.forEach { s ->
                if (s.values.size < 2) return@forEach
                val step = size.width / (s.values.size - 1)
                val path = Path()
                s.values.forEachIndexed { i, v ->
                    val point = Offset(i * step, mid - (v / range) * mid)
                    if (i == 0) path.moveTo(point.x, point.y) else path.lineTo(point.x, point.y)
                }
                drawPath(path, s.color, style = Stroke(width = 3f))
            }
        }
    }
}
