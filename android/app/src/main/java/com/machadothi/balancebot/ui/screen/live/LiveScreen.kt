package com.machadothi.balancebot.ui.screen.live

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.ui.components.ChartSeries
import com.machadothi.balancebot.ui.components.LineChart
import com.machadothi.balancebot.ui.components.RobotAttitude
import com.machadothi.balancebot.ui.components.ValueTile
import com.machadothi.balancebot.ui.theme.StateColors
import androidx.compose.foundation.layout.fillMaxWidth

@Composable
fun LiveScreen(viewModel: LiveViewModel = hiltViewModel()) {
    val connection by viewModel.connection.collectAsStateWithLifecycle()
    val live by viewModel.live.collectAsStateWithLifecycle()
    val history by viewModel.history.collectAsStateWithLifecycle()
    val s = live
    val colors = MaterialTheme.colorScheme

    Column(
        modifier = Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        if (connection !is ConnectionState.Connected || s == null) {
            Text("No data: connect to the robot first.", color = colors.error)
            return@Column
        }

        RobotAttitude(s, Modifier.fillMaxWidth())
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            ValueTile("Speed", "%.1f %%".format(s.speed), Modifier.weight(1f), detail = "of full wheel speed",
                accent = colors.secondary)
            ValueTile("Balance target", "%.2f°".format(s.setpoint), Modifier.weight(1f), detail = "after the speed loop",
                accent = colors.tertiary)
        }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            ValueTile("Motor output", "${s.output}", Modifier.weight(1f), detail = "of ±255",
                accent = if (kotlin.math.abs(s.output) > 200) StateColors.danger else colors.primary)
            ValueTile("Encoders", "${s.encoderLeft}", Modifier.weight(1f), detail = "right ${s.encoderRight}",
                accent = colors.primary)
        }

        LineChart(
            title = "Tilt, last minute (°)",
            minRange = 5f,
            series = listOf(
                ChartSeries("tilt", colors.primary, history.map { it.tilt }),
                ChartSeries("target", colors.tertiary, history.map { it.setpoint }),
            ),
        )
        LineChart(
            title = "Speed (%)",
            minRange = 10f,
            series = listOf(ChartSeries("speed", colors.secondary, history.map { it.speed })),
        )
    }
}
