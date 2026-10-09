package com.machadothi.balancebot.ui.theme

import androidx.compose.ui.graphics.Color

// Brand palette, shared with the launcher icon: night navy, teal, amber
val Navy = Color(0xFF0B1220)
val NavyLight = Color(0xFF111A2E)
val Slate = Color(0xFF1B2640)
val SlateLight = Color(0xFF2A3654)
val Teal = Color(0xFF2DD4BF)
val TealDark = Color(0xFF0D9488)
val DeepTeal = Color(0xFF134E5E)
val Amber = Color(0xFFF59E0B)
val Coral = Color(0xFFF43F5E)
val Mint = Color(0xFF34D399)
val Mist = Color(0xFFE2E8F0)
val Fog = Color(0xFF94A3B8)
val Paper = Color(0xFFF1F5F9)

/** Robot state colours, the same in both themes */
object StateColors {
    val balancing = Mint
    val tilted = Amber
    val off = Fog
    val danger = Coral
}
