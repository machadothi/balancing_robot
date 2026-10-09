package com.machadothi.balancebot.di

import com.machadothi.balancebot.repository.RobotRepository
import com.machadothi.balancebot.repository.RobotRepositoryImpl
import dagger.Binds
import dagger.Module
import dagger.hilt.InstallIn
import dagger.hilt.components.SingletonComponent

@Module
@InstallIn(SingletonComponent::class)
abstract class RepositoryModule {

    @Binds
    abstract fun bindsRobotRepository(impl: RobotRepositoryImpl): RobotRepository
}
