package com.machadothi.balancebot.ui.screen.live

import androidx.lifecycle.ViewModel
import com.machadothi.balancebot.repository.RobotRepository
import dagger.hilt.android.lifecycle.HiltViewModel
import javax.inject.Inject

@HiltViewModel
class LiveViewModel @Inject constructor(
    repository: RobotRepository,
) : ViewModel() {
    val connection = repository.connection
    val live = repository.live
    val history = repository.history
}
