package com.machadothi.balancebot.ui.screen.settings

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.model.ParamKind

@Composable
fun SettingsScreen(viewModel: SettingsViewModel = hiltViewModel()) {
    val connection by viewModel.connection.collectAsStateWithLifecycle()
    val items by viewModel.items.collectAsStateWithLifecycle()
    val message by viewModel.message.collectAsStateWithLifecycle()
    val connected = connection is ConnectionState.Connected

    LazyColumn(
        modifier = Modifier.fillMaxSize().padding(horizontal = 16.dp),
        verticalArrangement = Arrangement.spacedBy(8.dp),
    ) {
        item {
            Column(Modifier.padding(top = 16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                if (!connected) Text("Connect to the robot to read and change its settings.", color = MaterialTheme.colorScheme.error)
                Text(
                    "Changes apply immediately and last until the robot is switched off. " +
                        "To keep them, write them into the board config.",
                    style = MaterialTheme.typography.bodySmall,
                )
                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    OutlinedButton(onClick = viewModel::load, enabled = connected) { Text("Read again") }
                    OutlinedButton(onClick = viewModel::restoreDefaults, enabled = connected) { Text("Defaults") }
                }
                message?.let { Text(it, style = MaterialTheme.typography.bodySmall) }
            }
        }

        var lastGroup = ""
        items.forEach { item ->
            if (item.param.group != lastGroup) {
                lastGroup = item.param.group
                item(key = "group-$lastGroup") {
                    Text(item.param.group, style = MaterialTheme.typography.titleMedium, modifier = Modifier.padding(top = 8.dp))
                }
            }
            item(key = item.param.command) { ParamCard(item, connected, viewModel) }
        }
        item { Text("", modifier = Modifier.padding(bottom = 16.dp)) }
    }
}

@Composable
private fun ParamCard(item: ParamUi, connected: Boolean, viewModel: SettingsViewModel) {
    val p = item.param
    Card(modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(12.dp)) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Column(Modifier.weight(1f)) {
                    Text(p.label, style = MaterialTheme.typography.titleSmall)
                    Text(p.help, style = MaterialTheme.typography.bodySmall)
                }
                if (p.kind == ParamKind.SWITCH) {
                    Switch(
                        checked = item.current == "1",
                        enabled = connected && item.current != null,
                        onCheckedChange = { on -> viewModel.apply(p, if (on) "1" else "0") },
                    )
                } else {
                    Text(item.current ?: "–", style = MaterialTheme.typography.titleMedium)
                }
            }
            if (p.kind != ParamKind.SWITCH) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    OutlinedTextField(
                        value = item.edit,
                        onValueChange = { viewModel.onEdit(p, it) },
                        modifier = Modifier.weight(1f),
                        singleLine = true,
                        enabled = connected,
                        isError = item.error != null,
                        keyboardOptions = KeyboardOptions(
                            keyboardType = if (p.kind == ParamKind.PAIR) KeyboardType.Text else KeyboardType.Decimal
                        ),
                    )
                    Button(
                        onClick = { viewModel.apply(p) },
                        enabled = connected && item.error == null && item.edit != item.current,
                        modifier = Modifier.padding(start = 8.dp),
                    ) { Text("Set") }
                }
            }
            item.error?.let { Text(it, color = MaterialTheme.colorScheme.error, style = MaterialTheme.typography.bodySmall) }
        }
    }
}
