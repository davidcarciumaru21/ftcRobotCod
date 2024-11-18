package org.firstinspires.ftc.teamcode


import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import empireu.coyote.*
import org.firstinspires.ftc.teamcode.dashboard.CoyoteConfig.*
import org.firstinspires.ftc.teamcode.dashboard.LocalizerConfig
import org.firstinspires.ftc.teamcode.temp.ArmSystem
import java.io.BufferedInputStream
import java.io.File
import java.io.FileOutputStream
import java.lang.Integer.parseInt
import java.net.URL
import kotlin.math.PI
import kotlin.math.abs

@TeleOp(name = "TeleOp Servo", group = "TeleOp")
class TeleOpServo : LinearOpMode() {

    private lateinit var servoClaw: Servo
    private var pad = 0

    override fun runOpMode() {

        servoClaw = hardwareMap.get(Servo::class.java, "servoClaw")


        waitForStart()

        while (opModeIsActive()) {


            if (gamepad2.dpad_up && pad == 0) {
                pad = 1
                servoClaw.position = 0.25
                sleep(250)

            } else if (gamepad2.dpad_up && pad == 1) {
                pad = 0
                servoClaw.position = 0.0
                sleep(250)
            }


        }
    }
}








    @TeleOp(name = "TeleOp final")
    class teleopFinal : LinearOpMode() {
    
        private var pad = 0
        private var MOTOR_POWER_Lift = 1.0
        private var BratSensitivity = 0.75
        private lateinit var servoRoata: CRServo
        private lateinit var servoClaw: Servo
    
        private lateinit var Lift: DcMotorEx
        private lateinit var Unghi: DcMotorEx
    
        override fun runOpMode() {
    
            Lift = hardwareMap.get(DcMotorEx::class.java, "Lift")
            Unghi = hardwareMap.get(DcMotorEx::class.java, "Unghi")
    
            Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
            Unghi.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
    
            Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            Unghi.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
    
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            Unghi.setMode(DcMotor.RunMode.RUN_USING_ENCODER)
    
            Lift.setDirection(DcMotorSimple.Direction.FORWARD)
            Unghi.setDirection(DcMotorSimple.Direction.FORWARD)
    
            var center = 0
            var right = 0
            var init = 0
    
            val drive = MecanumDrive(hardwareMap)
            val lynxModules = hardwareMap.getAll(LynxModule::class.java)
            lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }
    
            servoRoata = hardwareMap.get(CRServo::class.java, "servoRoata")
            servoClaw = hardwareMap.get(Servo::class.java, "servoClaw")
            servoRoata.power = 0.0
    
            waitForStart()
    
            while (!isStopRequested) {
                lynxModules.forEach { it.clearBulkCache() }
    
                // Power profile based on bumper input
                val powerMultiplier = when {
                    gamepad1.left_bumper -> 0.25 // 25% power when left bumper is held
                    gamepad1.right_bumper -> 0.50 // 50% power when right bumper is held
                    else -> 1.0
                }
    
                drive.applyDrivePower(
                    Twist2d(
                        gamepad1.left_stick_y.toDouble() * powerMultiplier,
                        gamepad1.left_stick_x.toDouble() * powerMultiplier,
                        -gamepad1.right_stick_x.toDouble() * powerMultiplier
                    )
                )
                drive.update()
    
                // Servo Roata Control
                if (gamepad2.right_trigger > 0) {
                    servoRoata.power = 1.0
                } else if (gamepad2.left_trigger > 0) {
                    servoRoata.power = -1.0
                } else {
                    servoRoata.power = 0.0
                }
    
                // Lift and Unghi motor encoder feedback
                var encoderLift = Lift.currentPosition
                var encoderUnghi = Unghi.currentPosition
                telemetry.addData("unghi", encoderUnghi)
                telemetry.addData("lift", encoderLift)
    
                // Control unghi
                val crestere: Double = (abs(gamepad2.right_stick_y.toDouble()))
                telemetry.addData("crestere", crestere)
                if (gamepad2.right_stick_y > 0) {
                    Unghi.targetPosition = encoderUnghi + 80
                    Unghi.mode = DcMotor.RunMode.RUN_TO_POSITION
                    Unghi.power = crestere
                } else if (gamepad2.right_stick_y < 0) {
                    Unghi.targetPosition = encoderUnghi - 80
                    Unghi.mode = DcMotor.RunMode.RUN_TO_POSITION
                    Unghi.power = crestere
                }
    
                // Lift control
                if (gamepad2.left_bumper) {
                    Lift.targetPosition = encoderLift + 150
                    Lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    Lift.power = MOTOR_POWER_Lift
                } else if (gamepad2.right_bumper) {
                    Lift.targetPosition = encoderLift - 150
                    Lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    Lift.power = MOTOR_POWER_Lift
                }
                if (gamepad2.square){
                    Lift.setTargetPosition(-4035)
                    Unghi.setTargetPosition(-1573)
                    Lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    Lift.power = MOTOR_POWER_Lift
                    Unghi.mode = DcMotor.RunMode.RUN_TO_POSITION
                    Unghi.power = crestere
                }
                if (gamepad2.circle){
                    Lift.setTargetPosition(-7462)
                    Unghi.setTargetPosition(1302)
                    Lift.mode = DcMotor.RunMode.RUN_TO_POSITION
                    Lift.power = MOTOR_POWER_Lift
                    Unghi.mode = DcMotor.RunMode.RUN_TO_POSITION
                    Unghi.power = -crestere
                }

    
    
                // Servo Claw control
                if (center == 0 && init == 0) {
                    servoClaw.position = 0.71
                    center = 1
                    init = 1
                }
                if (gamepad2.dpad_right && right == 0) {
                    servoClaw.position = 1.0
                    center = 0
                    right = 1
                }
                if (gamepad2.dpad_left && center == 0 && right == 1) {
                    servoClaw.position = 0.71
                    center = 1
                    right = 0
                }
    
                telemetry.update()
            }
        }
    }


@TeleOp(name = "make america great again")
class maga : LinearOpMode() {
    private lateinit var motorViper: DcMotor

    override fun runOpMode() {
        motorViper = hardwareMap.get(DcMotor::class.java, "viper")

        motorViper.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motorViper.mode = DcMotor.RunMode.RUN_USING_ENCODER

        motorViper.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // Constants
        val feedForward = 0.1
        val maxRotationDegrees = 700.0
        val minRotationDegrees = -400.0
        val ticksPerRevolution = 751.8

        waitForStart()

        while (opModeIsActive()) {
            //servo









            val currentAngle = (motorViper.currentPosition / ticksPerRevolution) * 360


            val joystickInput = (-gamepad1.left_stick_y).toDouble()
            if ((currentAngle >= maxRotationDegrees && joystickInput > 0) ||
                (currentAngle <= minRotationDegrees && joystickInput < 0)) {
                motorViper.power = 0.0
            } else {
                motorViper.power = joystickInput
            }
            if(currentAngle <= 0){
                motorViper.power = feedForward
            }

            telemetry.addData("Current Angle", currentAngle)
            telemetry.update()
        }
    }
}







@TeleOp(name = "Motor Test", group = "Tuning")
class MotorTestOpMode: LinearOpMode() {
    override fun runOpMode() {
        val drive = MecanumDrive(hardwareMap)

        waitForStart()
        drive.motors.forEach {
            it.power = 0.1
            sleep(2500)
            it.power = 0.0
        }
    }
}

@TeleOp(name = "Manual Control")
class ManualOpMode : LinearOpMode() {
    override fun runOpMode() {
        FtcDashboard.getInstance().telemetryTransmissionInterval = 10
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = MecanumDrive(hardwareMap)
        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()
        while (!isStopRequested){
            lynxModules.forEach { it.clearBulkCache() }
            drive.applyDrivePower(

                Twist2d(
                    gamepad1.left_stick_y.toDouble(),
                    gamepad1.left_stick_x.toDouble(),
                    -gamepad1.right_stick_x.toDouble()
                )
            )

            drive.update()
        }
    }
}










@TeleOp(name = " Field oriented Manual Control")
class OrientedManualOpMode : LinearOpMode() {
    override fun runOpMode() {
        FtcDashboard.getInstance().telemetryTransmissionInterval = 10
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val drive = MecanumDrive(hardwareMap)
        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        waitForStart()
        while (!isStopRequested){
            lynxModules.forEach { it.clearBulkCache() }
            val inputWorld = Vector2d(gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble())
            val inputRobot = drive.position.rotation.inverse * inputWorld
            drive.applyDrivePower(

                Twist2d(
                    inputRobot.x,
                    inputRobot.y,
                    -gamepad1.right_stick_x.toDouble()
                )
            )

            drive.update()
        }
    }
}

@TeleOp(name = "Odometry Tuner", group = "Tuning")
class OdometryTunerOpMode : LinearOpMode() {
    override fun runOpMode() {
        telemetry = FtcDashboard.getInstance().telemetry

        val drive = MecanumDrive(hardwareMap)
        val localizer = Holo3WheelLocalizer(hardwareMap)

        waitForStart()

        var l = 0.0
        var r = 0.0
        var c = 0.0

        val angularDisp = 2.0 * PI * LocalizerConfig.TuningTurns

        while (opModeIsActive()) {
            drive.applyDrivePower(
                Twist2d(
                    0.0,
                    0.0,
                    -gamepad1.right_stick_x.toDouble()
                )
            )

            l += localizer.lEnc.readIncr()
            r += localizer.rEnc.readIncr()
            c += localizer.cEnc.readIncr()

            telemetry.addData("left distance", l / angularDisp)
            telemetry.addData("right distance", r / angularDisp)
            telemetry.addData("center distance", c / angularDisp)
            telemetry.update()
        }

        // Encoders are reading 0 around the time of stopping. I kept the calculation inside the loop because of this.
        // Why is this happening, and how do I fix it?
        // I am clearing the readout here so as to not cause confusion.
        telemetry.clear()
    }
}

@TeleOp(name = "Coyote Download", group = "Coyote")
class CoyoteDownload : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val writer: FileOutputStream

        try {
            if(File(robotCoyotePath).exists()){
                File(robotCoyotePath).delete()
            }

            File(robotCoyotePath).createNewFile()

            writer = FileOutputStream(robotCoyotePath)
        } catch (t: Throwable){
            telemetry.addData("Failed to open local storage", t)
            telemetry.update()
            sleep(30000)
            return
        }

        telemetry.addData("Target", robotCoyotePath)
        telemetry.update()

        waitForStart()

        try {
            val networkStream = BufferedInputStream(URL(CoyoteDownloadUrl).openStream())

            val buffer = ByteArray(1024)
            var read: Int
            var total = 0

            while (networkStream.read(buffer, 0, 1024).also { read = it } != -1) {
                writer.write(buffer, 0, read)
                total += read
            }

            networkStream.close()
            writer.close()

            telemetry.addData("Download size: ", total)
            telemetry.update()
            sleep(1000)
        }
        catch (t: Throwable){
            telemetry.addData("Network Error: ", t)
            telemetry.update()
            sleep(30000)
        }
    }
}

@TeleOp(name = "Coyote", group = "Coyote")
class CoyoteOpMode : LinearOpMode() {
    override fun runOpMode() {
        val dashboard = FtcDashboard.getInstance()
        dashboard.telemetryTransmissionInterval = 10
        telemetry.isAutoClear = true
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        val editorProject = loadRobotCoyoteProject()

        val savedNodeProject = editorProject.NodeProjects[NodeProjectName]
            ?: error("Failed to get node project $NodeProjectName")

        val drive = MecanumDrive(hardwareMap)
        val node: BehaviorNode

        try {
            node = loadNodeProject(
                savedNodeProject.RootNodes,
                BehaviorMapBuilder().also { b ->
                    b.add("Sequence", { ctx -> BehaviorSequenceNode(ctx.name, ctx.runOnce, ctx.childNodes) })
                    b.add("Selector", { ctx -> BehaviorSelectorNode(ctx.name, ctx.runOnce, ctx.childNodes) })
                    b.add("Success", { ctx -> BehaviorResultNode(ctx.name, ctx.runOnce, ctx.one(), BehaviorStatus.Success) })
                    b.add("Parallel", { ctx -> BehaviorParallelNode(ctx.name, ctx.runOnce, ctx.childNodes) })
                    b.add("Repeat", { ctx -> BehaviorRepeatNode(ctx.name, ctx.runOnce, parseInt(ctx.savedData), ctx.one()) })

                    b.add(
                        "Call",
                        { ctx -> BehaviorCallNode(ctx.name, ctx.runOnce) },

                        // The call nodes need a second pass (to search for the target node):
                        { ctx ->
                            val target = ctx.project.behaviors.firstOrNull { it.root.name == ctx.createContext.savedData }
                                ?: error("Failed to bind call node")

                            ctx.node.bind(target.root)
                        }
                    )

                    b.add("Motion", { ctx -> BehaviorMotionNode(ctx, editorProject, drive) })

                    registerNodes(b)
                }.build()
            ).behaviors
                .filter { it.root.name == EntryNodeName }
                .also {
                    if(it.isEmpty()) {
                        error("Failed to find root node $EntryNodeName")
                    }

                    if(it.size != 1) {
                        error("Ambiguous root node $EntryNodeName")
                    }
                }
                .first()
                .root
        }
        catch (t: Throwable) {
            telemetry.addData("ERROR", t.fillInStackTrace())
            telemetry.update()
            sleep(10000)
            return
        }

        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        val context = BehaviorContext()
        waitForStart()

        fun needsStop() = isStopRequested || gamepad1.left_bumper || gamepad1.right_bumper || gamepad2.left_bumper || gamepad2.right_bumper

        try {
            while (!needsStop()) {
                lynxModules.forEach { it.clearBulkCache() }

                val status = node.getStatus(context)

                if(status != BehaviorStatus.Running){
                    break
                }

                drive.update()
            }
        }
        finally {
            drive.resetPower()
        }
    }

    private fun registerNodes(builder: BehaviorMapBuilder) {
        builder.add("Arm", { ctx ->
            SystemNode(ctx) { ArmSystem(it, hardwareMap) }
        })
    }
}
