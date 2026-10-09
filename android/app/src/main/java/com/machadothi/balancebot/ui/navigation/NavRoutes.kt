package com.machadothi.balancebot.ui.navigation

import kotlinx.serialization.Serializable

object NavRoutes {

    @Serializable
    data object Connect

    @Serializable
    data object Drive

    @Serializable
    data object Live

    @Serializable
    data object Settings
}
