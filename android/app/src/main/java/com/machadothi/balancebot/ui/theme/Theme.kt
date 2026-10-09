package com.machadothi.balancebot.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

private val DarkColors = darkColorScheme(
    primary = Teal,
    onPrimary = Navy,
    primaryContainer = DeepTeal,
    onPrimaryContainer = Mist,
    secondary = Color(0xFF7DD3FC),
    onSecondary = Navy,
    tertiary = Amber,
    onTertiary = Navy,
    background = Navy,
    onBackground = Mist,
    surface = NavyLight,
    onSurface = Mist,
    surfaceVariant = Slate,
    onSurfaceVariant = Fog,
    surfaceContainer = NavyLight,
    surfaceContainerHigh = Slate,
    outline = SlateLight,
    error = Coral,
    onError = Color.White,
)

private val LightColors = lightColorScheme(
    primary = TealDark,
    onPrimary = Color.White,
    primaryContainer = Color(0xFFCCFBF1),
    onPrimaryContainer = DeepTeal,
    secondary = Color(0xFF0369A1),
    tertiary = Color(0xFFD97706),
    background = Paper,
    onBackground = Navy,
    surface = Color.White,
    onSurface = Navy,
    surfaceVariant = Color(0xFFE2E8F0),
    onSurfaceVariant = Color(0xFF475569),
    outline = Color(0xFFCBD5E1),
    error = Color(0xFFE11D48),
)

/**
 * The app's own colours in light and dark. Dynamic (wallpaper) colours are
 * deliberately not used: the brand palette matches the icon and the robot.
 */
@Composable
fun BalanceBotTheme(
    darkTheme: Boolean = isSystemInDarkTheme(),
    content: @Composable () -> Unit,
) {
    MaterialTheme(
        colorScheme = if (darkTheme) DarkColors else LightColors,
        typography = Typography,
        content = content,
    )
}
