package org.firstinspires.ftc.teamcode

import android.os.Environment
import com.google.blocks.ftcrobotcontroller.util.FileUtil
import com.google.gson.Gson
import empireu.coyote.JsonProject
import java.io.File

fun getRobotPath(file: String): String {
    val dir = Environment.getExternalStorageDirectory()
    val storagePath: String = dir.absolutePath
    return "$storagePath/$file"
}

val robotCoyotePath get() = getRobotPath("coyote.awoo")

fun loadRobotCoyoteProject(): JsonProject =
    Gson().fromJson(
        FileUtil.readFile(File(robotCoyotePath)),
        JsonProject::class.java
    )