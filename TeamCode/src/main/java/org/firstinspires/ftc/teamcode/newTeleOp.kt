package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.util.Angle
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable
import org.firstinspires.ftc.teamcode.util.storage.PoseStorage
import org.firstinspires.ftc.teamcode.util.storage.TrajStorage
import org.firstinspires.ftc.teamcode.util.storage.shooterMode

@TeleOp(name="shauryasinghteleop2")
class newTeleOp : LinearOpMode() {

    var shooterMode = shooterMode()
    var trajStorage = TrajStorage()

    //initialize ftc dashboard (online driver station)
    var dashboard: FtcDashboard = FtcDashboard.getInstance()
    var packet = TelemetryPacket()

    //3 shooters
    private var shooterMotor = Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE)
    private var intakeMotor1 = Motor(hardwareMap, "motor2", Motor.GoBILDA.RPM_312)
    private var intakeMotor2 = Motor(hardwareMap, "motor3", Motor.GoBILDA.RPM_312)
    private var linearSlide = Motor(hardwareMap, "motor4", Motor.GoBILDA.RPM_312)

    //four servos (flicker, liftings, gripper)
    private var flickerServo = SimpleServo(hardwareMap, "servo1")
    private var liftingServo1 = SimpleServo(hardwareMap, "servo2")
    private var liftingServo2 = SimpleServo(hardwareMap, "servo3")
    private var gripperServo = SimpleServo(hardwareMap, "servo4")

    private lateinit var digitalTouch: DigitalChannel

    private val targetPosition = Vector2d(72.0, 36.0)
    private var difference: Vector2d? = null

    //name gear ratio wheel radius and ticks for encoder cm conversion
    var GEAR_RATIO = 1.0 // for simulator
    var WHEEL_RADIUS = 1.0 // 5 cm
    var TICKS_PER_ROTATION = 103.6 // From NeveRest (for simulator)

    //calculate cm
    private var CM_PER_TICK = 2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS / TICKS_PER_ROTATION

    //calculate ticks from cm, return ticks
    private fun cmToTicks(cm: Double): Int {
        val cmTick = cm / CM_PER_TICK
        return cmTick.toInt()
    }

    enum class State {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL,
        SHOOTER_CONTROL,
        POWERSHOTS
    }

    private var currentState = State.DRIVER_CONTROL

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val drive = SampleMecanumDriveCancelable(hardwareMap)

        //set runmodes (PID for shooter, reg for intakes)
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl)
        intakeMotor1.setRunMode(Motor.RunMode.RawPower)
        intakeMotor2.setRunMode(Motor.RunMode.RawPower)
        linearSlide.setRunMode(Motor.RunMode.PositionControl)

        //set coefficients + feedforward (PID)
        shooterMotor.setVeloCoefficients(0.05, 0.01, 0.31)
        shooterMotor.setFeedforwardCoefficients(0.92, 0.47)
        linearSlide.positionCoefficient = 0.05

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel::class.java, "sensor_digital")
        digitalTouch.mode = DigitalChannel.Mode.INPUT

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        drive.poseEstimate = PoseStorage.currentPose

        waitForStart()

        if (isStopRequested) return

        shooterMotor.set(1.0)
        intakeMotor1.set(1.0)
        intakeMotor2.set(1.0)

        while (opModeIsActive() && !isStopRequested) {

            drive.update()
            val poseEstimate = drive.poseEstimate

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.x)
            telemetry.addData("y", poseEstimate.y)
            telemetry.addData("heading", poseEstimate.heading)
            telemetry.addData("mode", currentState)
            telemetry.update()

            when (currentState) {
                State.DRIVER_CONTROL -> {

                    val (x, y) = Vector2d(
                        (-gamepad1.left_stick_y).toDouble(),
                        (-gamepad1.left_stick_x).toDouble()
                    ).rotated(poseEstimate.heading)

                    //correct using PID, then put the target heading into the pose
                    drive.setWeightedDrivePower(
                        Pose2d(
                            x,
                            y,
                            (-gamepad1.right_stick_x).toDouble()
                        )
                    )

                    when {
                        gamepad1.left_trigger > 0.5 -> {
                            difference = targetPosition.minus(poseEstimate.vec())
                            val theta: Double = difference!!.angle()
                            val traj1 = drive.trajectoryBuilder(poseEstimate)
                                    .splineTo(Vector2d(0.0, drive.poseEstimate.y), Angle.normDelta(theta - drive.poseEstimate.heading))
                                    .addDisplacementMarker {
                                       shooterMode.servoSetGoal()
                                    }
                                    .build()
                            drive.followTrajectoryAsync(traj1)
                            currentState = State.SHOOTER_CONTROL
                        }
                        gamepad1.right_trigger > 0.5 -> {
                            var under = 0
                            while (under < 3) {
                                flickerServo.turnToAngle(1.0)
                                flickerServo.turnToAngle(0.0)
                                under++
                            }
                        }

                        gamepad1.a -> {
                            gripperServo.turnToAngle(0.0)
                        }
                        gamepad1.b -> {
                            val traj2 = drive.trajectoryBuilder(poseEstimate)
                                    .lineToSplineHeading(Pose2d(-23.0, -36.0, trajStorage.angleTheta))
                                    .addDisplacementMarker {
                                        shooterMode.servoSetGoal()
                                    }
                                    .build()
                            drive.followTrajectoryAsync(traj2)
                            currentState = State.POWERSHOTS
                        }
                        gamepad1.y -> {
                            gripperServo.turnToAngle(1.0)
                        }

                        gamepad1.right_bumper -> {
                            intakeMotor1.set(1.0)
                            intakeMotor2.set(1.0)
                        }
                        gamepad1.left_bumper -> {
                            intakeMotor1.set(-1.0)
                            intakeMotor2.set(-1.0)
                        }

                        gamepad1.dpad_up -> {
                            //set position for 20cm, 0 power, 13.6 tolerance
                            linearSlide.setTargetPosition(cmToTicks(20.0))
                            linearSlide.set(0.0)
                            linearSlide.setPositionTolerance(13.6)

                            //0.75 power until at target position
                            while (!linearSlide.atTargetPosition()) {
                                linearSlide.set(0.75)
                            }

                            //stop when done
                            gripperServo.turnToAngle(0.0) //drop it off
                            linearSlide.stopMotor()
                        }
                        gamepad1.dpad_down -> {
                            //set position for -20cm (relative), 0 power, 13.6 tolerence
                            linearSlide.setTargetPosition(cmToTicks(0.0))
                            linearSlide.set(0.0)
                            linearSlide.setPositionTolerance(13.6)

                            //continue until limit switch state is true (detects slide)
                            while (!digitalTouch.state) {
                                linearSlide.set(0.75)
                            }

                            //stop motor
                            linearSlide.stopMotor()
                        }
                        gamepad1.dpad_left -> {
                            drive.turn(drive.poseEstimate.heading - 90)
                            currentState = State.AUTOMATIC_CONTROL
                        }
                        gamepad1.dpad_right -> {
                            drive.turn(drive.poseEstimate.heading + 90)
                            currentState = State.AUTOMATIC_CONTROL
                        }
                    }

                }
                State.AUTOMATIC_CONTROL -> {
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing()
                        currentState = State.DRIVER_CONTROL
                    }

                    // If drive finishes its task, put control to the driver
                    if (!drive.isBusy) {
                        currentState = State.DRIVER_CONTROL
                        break
                    }
                }
                State.SHOOTER_CONTROL -> {
                    if (gamepad1.x) {
                        drive.cancelFollowing()
                        currentState = State.DRIVER_CONTROL
                    }

                    // If drive finishes its task, put control to the driver
                    if (!drive.isBusy) {
                        liftingServo1.turnToAngle(0.0)
                        liftingServo2.turnToAngle(0.0)
                        currentState = State.DRIVER_CONTROL
                        break
                    }
                }
                State.POWERSHOTS -> {
                    if (gamepad1.x) {
                        drive.cancelFollowing()
                        currentState = State.DRIVER_CONTROL
                    }

                    // If drive finishes its task, put control to the driver
                    if (!drive.isBusy) {
                        flickerServo.turnToAngle(1.0)
                        flickerServo.turnToAngle(0.0)
                        drive.turn(trajStorage.angleTheta1)
                        flickerServo.turnToAngle(1.0)
                        flickerServo.turnToAngle(0.0)
                        drive.turn(trajStorage.angleTheta2)
                        flickerServo.turnToAngle(1.0)
                        flickerServo.turnToAngle(0.0)
                        currentState = State.DRIVER_CONTROL
                        break
                    }
                }
            }
        }
    }
}