package com.machadothi.balancebot.ui.components

import android.graphics.BitmapFactory
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.unit.dp
import com.machadothi.balancebot.data.camera.MjpegReader
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.withContext
import java.net.HttpURLConnection
import java.net.URL
import kotlin.coroutines.coroutineContext

/** Live MJPEG from the robot's camera; reconnects by itself while shown */
@Composable
fun CameraView(url: String, modifier: Modifier = Modifier) {
    var frame by remember { mutableStateOf<ImageBitmap?>(null) }
    var status by remember { mutableStateOf("connecting to the camera ...") }

    LaunchedEffect(url) {
        while (isActive) {
            try {
                stream(url) { bitmap ->
                    frame = bitmap
                    status = ""
                }
            } catch (e: Exception) {
                if (!coroutineContext.isActive) throw e
                status = "camera: ${e.message ?: e.javaClass.simpleName}; retrying"
            }
            delay(RETRY_MS)
        }
    }

    Box(
        modifier = modifier
            .aspectRatio(4f / 3f)
            .clip(RoundedCornerShape(12.dp))
            .background(Color.Black),
        contentAlignment = Alignment.Center,
    ) {
        frame?.let { Image(it, contentDescription = "robot camera", Modifier.fillMaxSize(), contentScale = ContentScale.Fit) }
        if (status.isNotEmpty()) {
            Text(status, color = Color.White, style = MaterialTheme.typography.bodySmall, modifier = Modifier.padding(12.dp))
        }
    }
}

private suspend fun stream(url: String, onFrame: (ImageBitmap) -> Unit) = withContext(Dispatchers.IO) {
    val connection = URL(url).openConnection() as HttpURLConnection
    connection.connectTimeout = 5000
    connection.readTimeout = 5000
    try {
        val reader = MjpegReader(connection.inputStream.buffered())
        while (isActive) {
            val jpeg = reader.nextFrame()
            BitmapFactory.decodeByteArray(jpeg, 0, jpeg.size)?.let { bitmap ->
                withContext(Dispatchers.Main) { onFrame(bitmap.asImageBitmap()) }
            }
        }
    } finally {
        connection.disconnect()
    }
}

private const val RETRY_MS = 2000L
