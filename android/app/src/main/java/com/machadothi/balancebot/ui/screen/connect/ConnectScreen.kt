package com.machadothi.balancebot.ui.screen.connect

import android.Manifest
import android.content.Context
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
import androidx.compose.foundation.layout.size
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
import com.machadothi.balancebot.model.RobotDevice

/** Android 12+: "Nearby devices" (connect and scan). Older: location, which BLE scans need. */
private val PERMISSIONS: Array<String> =
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
        arrayOf(Manifest.permission.BLUETOOTH_CONNECT, Manifest.permission.BLUETOOTH_SCAN)
    } else {
        arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
    }

private fun allGranted(context: Context) = PERMISSIONS.all {
    ContextCompat.checkSelfPermission(context, it) == PackageManager.PERMISSION_GRANTED
}

@Composable
fun ConnectScreen(
    onConnected: () -> Unit,
    viewModel: ConnectViewModel = hiltViewModel(),
) {
    val context = LocalContext.current
    val connection by viewModel.connection.collectAsStateWithLifecycle()
    val paired by viewModel.paired.collectAsStateWithLifecycle()
    val nearby by viewModel.nearby.collectAsStateWithLifecycle()
    val scanning by viewModel.scanning.collectAsStateWithLifecycle()
    val scanError by viewModel.scanError.collectAsStateWithLifecycle()
    val bluetoothOn by viewModel.bluetoothOn.collectAsStateWithLifecycle()

    var granted by remember { mutableStateOf(allGranted(context)) }
    val launcher = rememberLauncherForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {
        granted = allGranted(context)
    }

    LaunchedEffect(granted) {
        if (granted) {
            viewModel.refresh()
            if (connection !is ConnectionState.Connected) viewModel.scan()
        }
    }
    var wasConnecting by remember { mutableStateOf(false) }
    LaunchedEffect(connection) {
        if (connection is ConnectionState.Connecting) wasConnecting = true
        if (connection is ConnectionState.Connected && wasConnecting) {
            wasConnecting = false
            onConnected()
        }
    }
    val busy = connection is ConnectionState.Connecting

    LazyColumn(
        modifier = Modifier.fillMaxSize().padding(horizontal = 16.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        item {
            Column(Modifier.padding(top = 16.dp), verticalArrangement = Arrangement.spacedBy(12.dp)) {
                when (val c = connection) {
                    is ConnectionState.Connected -> {
                        Text("Connected to ${c.device.name} (${c.device.kind.label})", style = MaterialTheme.typography.titleMedium)
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
                        Text("The app needs the Bluetooth (\"Nearby devices\") permission to find and reach the robot.")
                        Button(onClick = { launcher.launch(PERMISSIONS) }) { Text("Allow Bluetooth") }
                    }
                    !bluetoothOn -> {
                        Text("Bluetooth is off. Switch it on, then refresh.")
                        OutlinedButton(onClick = viewModel::refresh) { Text("Refresh") }
                    }
                }
            }
        }

        if (granted && bluetoothOn) {
            item {
                SectionHeader("Atom (BLE + camera)") {
                    if (scanning) {
                        CircularProgressIndicator(Modifier.size(24.dp))
                    } else {
                        OutlinedButton(onClick = viewModel::scan) { Text("Scan") }
                    }
                }
                Text(
                    "No pairing needed. The robot shows up as \"balancing robot\" while it is on." +
                        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S) " Location must be on for the scan." else "",
                    style = MaterialTheme.typography.bodySmall,
                )
                scanError?.let { Text(it, color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall) }
                if (!scanning && nearby.isEmpty()) {
                    Text("Nothing found.", style = MaterialTheme.typography.bodySmall)
                }
            }
            items(nearby, key = { "ble-" + it.address }) { DeviceCard(it, enabled = !busy) { viewModel.connect(it) } }

            item {
                SectionHeader("Paired (HC-06)") {
                    OutlinedButton(onClick = viewModel::refresh) { Text("Refresh") }
                }
                Text(
                    "Pair the HC-06 first in the phone's Bluetooth settings: it shows up as \"balancing robot\", PIN 1234.",
                    style = MaterialTheme.typography.bodySmall,
                )
            }
            items(paired, key = { "bt-" + it.address }) { DeviceCard(it, enabled = !busy) { viewModel.connect(it) } }
            item { Text("", Modifier.padding(bottom = 16.dp)) }
        }
    }
}

@Composable
private fun SectionHeader(title: String, action: @Composable () -> Unit) {
    Row(
        modifier = Modifier.fillMaxWidth().padding(top = 8.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(title, style = MaterialTheme.typography.titleMedium)
        action()
    }
}

@Composable
private fun DeviceCard(device: RobotDevice, enabled: Boolean, onClick: () -> Unit) {
    Card(modifier = Modifier.fillMaxWidth().clickable(enabled = enabled, onClick = onClick)) {
        Column(Modifier.padding(16.dp)) {
            Text(
                device.name + if (device.looksLikeRobot) "  (robot?)" else "",
                style = MaterialTheme.typography.titleSmall,
            )
            Text(device.address, style = MaterialTheme.typography.bodySmall)
        }
    }
}
