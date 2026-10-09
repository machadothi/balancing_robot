package com.machadothi.balancebot.ui.navigation

import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Info
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.Icon
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.navigation.NavDestination.Companion.hasRoute
import androidx.navigation.NavGraph.Companion.findStartDestination
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import com.machadothi.balancebot.ui.components.RobotTopBar
import com.machadothi.balancebot.ui.screen.connect.ConnectScreen
import com.machadothi.balancebot.ui.screen.drive.DriveScreen
import com.machadothi.balancebot.ui.screen.live.LiveScreen
import com.machadothi.balancebot.ui.screen.settings.SettingsScreen
import kotlin.reflect.KClass

private data class Tab(val route: Any, val type: KClass<*>, val label: String, val icon: ImageVector)

private val tabs = listOf(
    Tab(NavRoutes.Connect, NavRoutes.Connect::class, "Connect", Icons.Default.Home),
    Tab(NavRoutes.Drive, NavRoutes.Drive::class, "Drive", Icons.Default.PlayArrow),
    Tab(NavRoutes.Live, NavRoutes.Live::class, "Live", Icons.Default.Info),
    Tab(NavRoutes.Settings, NavRoutes.Settings::class, "Settings", Icons.Default.Settings),
)

@Composable
fun NavigationScreen() {
    val navController = rememberNavController()
    val backStack by navController.currentBackStackEntryAsState()
    val destination = backStack?.destination

    fun open(route: Any) = navController.navigate(route) {
        popUpTo(navController.graph.findStartDestination().id) { saveState = true }
        launchSingleTop = true
        restoreState = true
    }

    Scaffold(
        modifier = Modifier.fillMaxSize(),
        topBar = { RobotTopBar() },
        bottomBar = {
            NavigationBar {
                tabs.forEach { tab ->
                    NavigationBarItem(
                        selected = destination?.hasRoute(tab.type) == true,
                        onClick = { open(tab.route) },
                        icon = { Icon(tab.icon, contentDescription = tab.label) },
                        label = { Text(tab.label) },
                    )
                }
            }
        },
    ) { innerPadding ->
        NavHost(
            navController = navController,
            startDestination = NavRoutes.Connect,
            modifier = Modifier.padding(innerPadding),
        ) {
            composable<NavRoutes.Connect> { ConnectScreen(onConnected = { open(NavRoutes.Drive) }) }
            composable<NavRoutes.Drive> { DriveScreen() }
            composable<NavRoutes.Live> { LiveScreen() }
            composable<NavRoutes.Settings> { SettingsScreen() }
        }
    }
}
