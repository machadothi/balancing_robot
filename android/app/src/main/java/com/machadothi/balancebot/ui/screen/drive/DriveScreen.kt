package com.machadothi.balancebot.ui.screen.drive

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.model.LinkKind
import com.machadothi.balancebot.ui.components.CameraView
import com.machadothi.balancebot.ui.components.Joystick
import com.machadothi.balancebot.ui.components.RobotAttitude
import com.machadothi.balancebot.ui.components.ValueTile
import com.machadothi.balancebot.ui.theme.StateColors

@Composable
fun DriveScreen(viewModel: DriveViewModel = hiltViewModel()) {
    val connection by viewModel.connection.collectAsStateWithLifecycle()
    val live by viewModel.live.collectAsStateWithLifecycle()
    val driveEnabled by viewModel.driveEnabled.collectAsStateWithLifecycle()
    val maxSpeed by viewModel.maxSpeed.collectAsStateWithLifecycle()
    val maxTurn by viewModel.maxTurn.collectAsStateWithLifecycle()
    val command by viewModel.command.collectAsStateWithLifecycle()
    val message by viewModel.message.collectAsStateWithLifecycle()
    val cameraUrl by viewModel.cameraUrl.collectAsStateWithLifecycle()
    val showCamera by viewModel.showCamera.collectAsStateWithLifecycle()
    val connected = connection is ConnectionState.Connected

    Column(
        modifier = Modifier
            .fillMaxSize()
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        if (!connected) {
            Text("Connect to the robot first (Connect tab).", color = MaterialTheme.colorScheme.error)
        }

        cameraUrl?.let { url ->
            Row(verticalAlignment = Alignment.CenterVertically) {
                Switch(checked = showCamera, onCheckedChange = viewModel::setShowCamera)
                Text("Camera", style = MaterialTheme.typography.titleSmall, modifier = Modifier.padding(start = 12.dp))
            }
            if (showCamera) CameraView(url, Modifier.fillMaxWidth())
        } ?: run {
            if ((connection as? ConnectionState.Connected)?.device?.kind == LinkKind.BLE) {
                Text(
                    "Camera: waiting for the Atom's Wi-Fi. Set the network in Settings → Camera Wi-Fi.",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }

        RobotAttitude(live, Modifier.fillMaxWidth())
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            ValueTile("Speed", live?.let { "%.1f %%".format(it.speed) } ?: "–", Modifier.weight(1f),
                accent = MaterialTheme.colorScheme.secondary)
            ValueTile("Motors", live?.let { if (it.enabled) "ON" else "OFF" } ?: "–", Modifier.weight(1f),
                accent = if (live?.enabled == true) StateColors.balancing else StateColors.off)
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = viewModel::enable, enabled = connected, modifier = Modifier.weight(1f)) {
                Text("Enable")
            }
            Button(
                onClick = viewModel::stop,
                enabled = connected,
                modifier = Modifier.weight(1f),
                colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error),
            ) { Text("Stop") }
        }

        Row(verticalAlignment = Alignment.CenterVertically) {
            Switch(checked = driveEnabled, onCheckedChange = viewModel::setDriveEnabled, enabled = connected)
            Column(Modifier.padding(start = 12.dp)) {
                Text("Drive with the stick", style = MaterialTheme.typography.titleSmall)
                Text(
                    "The robot stops by itself 1 s after the commands stop (link lost, app closed).",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }

        Joystick(
            modifier = Modifier.fillMaxWidth().padding(horizontal = 24.dp),
            enabled = connected && driveEnabled,
            baseColor = MaterialTheme.colorScheme.primary,
            knobColor = MaterialTheme.colorScheme.tertiary,
            onMove = viewModel::onStick,
        )
        Text(
            "Sending: speed %+.1f %% · turn %+.0f".format(command.speed, command.turn),
            style = MaterialTheme.typography.bodyMedium,
        )

        Text("Max speed: %.0f %%".format(maxSpeed), style = MaterialTheme.typography.labelLarge)
        Slider(value = maxSpeed, onValueChange = viewModel::setMaxSpeed, valueRange = 2f..40f)
        Text("Max turn: %.0f".format(maxTurn), style = MaterialTheme.typography.labelLarge)
        Slider(value = maxTurn, onValueChange = viewModel::setMaxTurn, valueRange = 5f..100f)

        message?.let { Text(it, color = MaterialTheme.colorScheme.error) }
    }
}
