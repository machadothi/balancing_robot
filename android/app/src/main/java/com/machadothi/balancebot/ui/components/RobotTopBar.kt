package com.machadothi.balancebot.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import com.machadothi.balancebot.ui.theme.DeepTeal
import com.machadothi.balancebot.ui.theme.Navy
import com.machadothi.balancebot.ui.theme.StateColors
import com.machadothi.balancebot.ui.theme.TealDark
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.lifecycle.ViewModel
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.viewModelScope
import com.machadothi.balancebot.model.ConnectionState
import com.machadothi.balancebot.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import kotlinx.coroutines.launch
import javax.inject.Inject

@HiltViewModel
class TopBarViewModel @Inject constructor(
    private val repository: RobotRepository,
) : ViewModel() {
    val connection = repository.connection
    val live = repository.live

    fun stop() {
        viewModelScope.launch { repository.stop() }
    }
}

/** Connection state and robot state on every screen, plus an always-reachable STOP */
@Composable
fun RobotTopBar(viewModel: TopBarViewModel = hiltViewModel()) {
    val connection by viewModel.connection.collectAsStateWithLifecycle()
    val live by viewModel.live.collectAsStateWithLifecycle()

    val (dot, status) = when (val c = connection) {
        ConnectionState.Disconnected -> StateColors.off to "not connected"
        is ConnectionState.Connecting -> StateColors.tilted to "connecting to ${c.device.name} ..."
        is ConnectionState.Failed -> StateColors.danger to c.message
        is ConnectionState.Connected -> {
            val s = live
            when {
                s == null -> StateColors.tilted to c.device.name
                s.enabled -> StateColors.balancing to "balancing · ${c.device.name}"
                else -> StateColors.off to "motors off · ${c.device.name}"
            }
        }
    }

    Box(Modifier.fillMaxWidth().background(Brush.horizontalGradient(listOf(Navy, DeepTeal, TealDark)))) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .statusBarsPadding()
                .padding(horizontal = 16.dp, vertical = 10.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween,
        ) {
            Column(Modifier.weight(1f)) {
                Text("Balance Bot", style = MaterialTheme.typography.titleLarge, fontWeight = FontWeight.Bold, color = Color.White)
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Box(Modifier.size(8.dp).clip(CircleShape).background(dot))
                    Text(
                        status,
                        modifier = Modifier.padding(start = 6.dp),
                        style = MaterialTheme.typography.bodySmall,
                        color = Color.White.copy(alpha = 0.85f),
                        maxLines = 2,
                    )
                }
            }
            if (connection is ConnectionState.Connected) {
                Button(
                    onClick = viewModel::stop,
                    colors = ButtonDefaults.buttonColors(containerColor = StateColors.danger, contentColor = Color.White),
                ) { Text("STOP", fontWeight = FontWeight.Bold) }
            }
        }
    }
}
