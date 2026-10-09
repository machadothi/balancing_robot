package com.machadothi.balancebot.model

/** A network the Atom sees (AT+WIFISCAN?) */
data class WifiNetwork(val ssid: String, val rssi: Int) {
    /** 0..4 bars, like the phone's Wi-Fi icon */
    val bars: Int get() = when {
        rssi >= -55 -> 4
        rssi >= -65 -> 3
        rssi >= -75 -> 2
        rssi >= -85 -> 1
        else -> 0
    }

    companion object {
        /** "<rssi> <ssid>" entries separated by tabs, strongest first (atom/main/camera_http.c) */
        fun parseList(value: String): List<WifiNetwork> = value.split('\t').mapNotNull { entry ->
            val space = entry.indexOf(' ')
            if (space <= 0) return@mapNotNull null
            val rssi = entry.substring(0, space).toIntOrNull() ?: return@mapNotNull null
            val ssid = entry.substring(space + 1)
            if (ssid.isEmpty()) null else WifiNetwork(ssid, rssi)
        }
    }
}
