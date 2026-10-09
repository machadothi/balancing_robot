package com.machadothi.balancebot.ui.screen.connect

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.core.content.ContextCompat
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.machadothi.balancebot.model.ConnectionState

@Composable
fun ConnectScreen(
    onConnected: () -> Unit,
    viewModel: ConnectViewModel = hiltViewModel(),
) {
    val context = LocalContext.current
    val connection by viewModel.connection.collectAsStateWithLifecycle()
    val devices by viewModel.devices.collectAsStateWithLifecycle()
    val bluetoothOn by viewModel.bluetoothOn.collectAsStateWithLifecycle()

    // Android 12+ asks for "Nearby devices" before paired devices can be listed
    val needsPermission = Build.VERSION.SDK_INT >= Build.VERSION_CODES.S
    var granted by remember {
        mutableStateOf(
            !needsPermission || ContextCompat.checkSelfPermission(
                context, Manifest.permission.BLUETOOTH_CONNECT
            ) == PackageManager.PERMISSION_GRANTED
        )
    }
    val launcher = rememberLauncherForActivityResult(ActivityResultContracts.RequestPermission()) {
        granted = it
        if (it) viewModel.refresh()
    }

    LaunchedEffect(granted) { if (granted) viewModel.refresh() }
    var wasConnecting by remember { mutableStateOf(false) }
    LaunchedEffect(connection) {
        if (connection is ConnectionState.Connecting) wasConnecting = true
        if (connection is ConnectionState.Connected && wasConnecting) {
            wasConnecting = false
            onConnected()
        }
    }

    Column(
        modifier = Modifier.fillMaxSize().padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        when (val c = connection) {
            is ConnectionState.Connected -> {
                Text("Connected to ${c.device.name}", style = MaterialTheme.typography.titleMedium)
                OutlinedButton(onClick = viewModel::disconnect) { Text("Disconnect") }
            }
            is ConnectionState.Connecting -> Row(verticalAlignment = Alignment.CenterVertically) {
                CircularProgressIndicator(modifier = Modifier.padding(end = 12.dp))
                Text("Connecting to ${c.device.name} ...")
            }
            is ConnectionState.Failed -> Text(c.message, color = MaterialTheme.colorScheme.error)
            ConnectionState.Disconnected -> Unit
        }

        when {
            !granted -> {
                Text("The app needs the Bluetooth (\"Nearby devices\") permission to reach the robot.")
                Button(onClick = { launcher.launch(Manifest.permission.BLUETOOTH_CONNECT) }) {
                    Text("Allow Bluetooth")
                }
            }
            !bluetoothOn -> {
                Text("Bluetooth is off. Switch it on, then refresh.")
                OutlinedButton(onClick = viewModel::refresh) { Text("Refresh") }
            }
            else -> {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Text("Paired devices", style = MaterialTheme.typography.titleMedium)
                    OutlinedButton(onClick = viewModel::refresh) { Text("Refresh") }
                }
                Text(
                    "Pair the robot first in the phone's Bluetooth settings: it shows up as \"balancing robot\", PIN 1234.",
                    style = MaterialTheme.typography.bodySmall,
                )
                LazyColumn(verticalArrangement = Arrangement.spacedBy(8.dp)) {
                    items(devices, key = { it.address }) { device ->
                        Card(
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable(enabled = connection !is ConnectionState.Connecting) {
                                    viewModel.connect(device)
                                },
                        ) {
                            Column(Modifier.padding(16.dp)) {
                                Text(
                                    device.name + if (device.looksLikeRobot) "  (robot?)" else "",
                                    style = MaterialTheme.typography.titleSmall,
                                )
                                Text(device.address, style = MaterialTheme.typography.bodySmall)
                            }
                        }
                    }
                }
            }
        }
    }
}
