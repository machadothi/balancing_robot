package com.machadothi.balancebot

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import com.machadothi.balancebot.ui.navigation.NavigationScreen
import com.machadothi.balancebot.ui.theme.BalanceBotTheme
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            BalanceBotTheme {
                NavigationScreen()
            }
        }
    }
}
