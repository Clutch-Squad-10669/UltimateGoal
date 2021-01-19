package org.firstinspires.ftc.teamcode

//imports
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.util.Angle
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.util.EOCVtests.bounceBaccPipeline
import org.firstinspires.ftc.teamcode.util.EOCVtests.distanceCenterLUT
import org.firstinspires.ftc.teamcode.util.EOCVtests.distanceWidthLUT
import org.firstinspires.ftc.teamcode.util.storage.PoseStorage
import org.firstinspires.ftc.teamcode.util.storage.TrajStorage
import org.firstinspires.ftc.teamcode.util.storage.shooterMode
import org.openftc.easyopencv.*
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

/*
   This is the Team10669 clutch autonomous code for UG 2020-2021.
   It includes a contour-based ring detector, finite state machines, and PID control for various motors
   Odometry is done through the roadrunner library @see <a href="https://learnroadrunner.com">learnroadrunner</a>
   The vision pipeline, servo control, and PID control is done through FTClib  @see <a href="https://docs.ftclib.org/ftclib/">FTClib</a>
   (new version is in kotlin, contact for older java version)
*/

//declare autonomous
@Autonomous(name = "Start1Auto")
class Start1 : LinearOpMode() {

    //import trajectory storage (contains trajectory files - uses roadrunner), import shootermode (for shooter alignment)
    private var trajStorage = TrajStorage()
    private var shooterMode = shooterMode()

    //1 shooter, 2 intake, 1 linear slide
    private var shooterMotor = Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE)
    private var intakeMotor1 = Motor(hardwareMap, "motor2", Motor.GoBILDA.RPM_312)
    private var intakeMotor2 = Motor(hardwareMap, "motor3", Motor.GoBILDA.RPM_312)
    private var linearSlide = Motor(hardwareMap, "motor4", 288.0, 125.0) //rev hex

    //four servos (flicker, lifting, gripper)
    private var flickerServo = SimpleServo(hardwareMap, "servo1")
    private var liftingServo1 = SimpleServo(hardwareMap, "servo2")
    private var liftingServo2 = SimpleServo(hardwareMap, "servo3")
    private var gripperServo = SimpleServo(hardwareMap, "servo4")

    //touch sensor
    private lateinit var digitalTouch: DigitalChannel

    //write toRadians (for rr pose)
    val Double.toRadians get() = (Math.toRadians(this))

    //initialize ftc dashboard (online driver station)
    var dashboard: FtcDashboard = FtcDashboard.getInstance()
    private var packet = TelemetryPacket()

    //initialize the pipeline and  camera

    /*
    he protecc
    he atacc
    but most importantly
    he bouncebacc
     */
    private lateinit var pipeline: bounceBaccPipeline
    private var camera: OpenCvCamera = if (USING_WEBCAM) configureWebCam()
    else configurePhoneCamera()

    private val cameraMonitorViewId: Int = hardwareMap
        .appContext
        .resources
        .getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName,
        )

    //configure the phone camera, and webcam
    private fun configurePhoneCamera(): OpenCvInternalCamera2 = OpenCvCameraFactory.getInstance()
        .createInternalCamera2(
            OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId
        )

    private fun configureWebCam(): OpenCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(
        hardwareMap.get(WebcamName::class.java, WEBCAM_NAME), cameraMonitorViewId,
    )

    //get info about the ring (from pipeline)
    private fun getRingInfo(): Vector2d {
        val headingRing = distanceCenterLUT.get(pipeline.getRectCenter().x)
        val distanceRing = distanceWidthLUT.get(pipeline.getRectWidth().toDouble())
        return  Vector2d (headingRing, distanceRing)
    }

    //variable for shooter cycle loop (end of auto)
    var loopNumShooter = 0

    //create a state enum for our finite state machine
    enum class State {
        FOUR, ONE, ZERO, BOUNCEBACC, SHOOTER_CONTROL, RESET_DRIVE_POS
    }

    //start the op mode
    @Throws(InterruptedException::class)
    override fun runOpMode() {

        //hardwareMap
        val drive = SampleMecanumDrive(hardwareMap)

        //trajectory follower (makes life easier)
        fun followTrajectories(vararg trajectories: Trajectory) {
            for (trajectory in trajectories) {
                drive.followTrajectory(trajectory)
            }
        }

        //set pipelines
        camera.setPipeline(bounceBaccPipeline(telemetry, DEBUG).apply { pipeline = this })

        //set parameters
        bounceBaccPipeline.CAMERA_WIDTH = CAMERA_WIDTH
        bounceBaccPipeline.HORIZON = HORIZON

        //start streaming to driver station
        camera.openCameraDeviceAsync {
            camera.startStreaming(
                CAMERA_WIDTH,
                CAMERA_HEIGHT,
                OpenCvCameraRotation.UPRIGHT
            )
        }

        //start streaming to ftc dash
        FtcDashboard.getInstance().startCameraStream(camera, 10.0)

        //set current state for zero
        var state = State.ZERO

        //method to drive to ring pos
        fun driveToRingPosition() {
            val goToRingPosition = drive.trajectoryBuilder(
                Pose2d(drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading)) //set pose to estimate
                .lineToSplineHeading(Pose2d(
                    (sin(getRingInfo().y) * getRingInfo().x), //find opposite side
                    (cos(getRingInfo().y) * getRingInfo().x), //find adjacent size
                    (getRingInfo().y - 180.0).toRadians)) //find heading - 180 (intake on the back)
                .build()
            drive.followTrajectory(goToRingPosition) //follow traj
            state = State.BOUNCEBACC //change state
        }

        fun resetDrivePosition(){
            val resetDrivePos = drive.trajectoryBuilder(
                Pose2d(drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading))
                .lineToSplineHeading(Pose2d(60.0, 60.0, 0.0))
                .build()
            drive.followTrajectory(resetDrivePos)
            state = State.RESET_DRIVE_POS
        }

        fun driveToShoot() {
            val targetPosition = Vector2d(72.0, 36.0) //high goal pos
            val difference: Vector2d = targetPosition.minus(drive.poseEstimate.vec()) //difference
            val theta: Double = difference.angle() //angle of difference
            val x = max(0.0, drive.poseEstimate.x) //sets x to same if its already behind line
            val driveToBehindLine = drive.trajectoryBuilder(
                Pose2d(drive.poseEstimate.x, drive.poseEstimate.y, drive.poseEstimate.heading)) //set pose to estimate
                .splineTo(
                    Vector2d(x, drive.poseEstimate.y), //x is line, y is current y
                    Angle.normDelta(theta - drive.poseEstimate.heading)
                )
                .addDisplacementMarker {
                    shooterMode.servoSetGoal()
                }
                .build()
            drive.followTrajectory(driveToBehindLine)
            state = State.SHOOTER_CONTROL

        }

        //get the height from the pipeline (FOUR, ONE, or ZERO), set the state depending on the height
        while (!isStarted) {
            // ^ this is very important, it makes sure that the detector is running until the opmode is started
            // the ring stack is often changed after the init is started, therefore this gives the last detected value
            state = when (pipeline.height) {
                bounceBaccPipeline.Height.ZERO -> State.ZERO
                bounceBaccPipeline.Height.ONE -> State.ONE
                bounceBaccPipeline.Height.FOUR -> State.FOUR
            }
        }

        // the usual wait for start, as well as if stop requested 
        waitForStart()
        if (isStopRequested) return

        //sets powers
        //shooterMotor.set(1.0)
        //intakeMotor.set(1.0)

        when (state) {
            State.ZERO -> {
                //add telemetry for state (testing)
                packet.put("current state", state)
                dashboard.sendTelemetryPacket(packet)

                followTrajectories(
                    trajStorage.a1red1,
                    trajStorage.a1red2,
                    trajStorage.a1red3,
                    trajStorage.a1red4,
                    trajStorage.a1red5,
                    trajStorage.a1red6,
                    trajStorage.a1red7
                )
                driveToRingPosition()
            }

            State.ONE -> {
                //add telemetry for state (testing)
                packet.put("current state", state)
                dashboard.sendTelemetryPacket(packet)

                followTrajectories(
                    trajStorage.trajectoryB1Red1,
                    trajStorage.trajectoryA1Red2,
                    trajStorage.trajectoryA1Red3,
                    trajStorage.trajectoryA1Red4,
                    trajStorage.trajectoryB1Red5,
                    trajStorage.trajectoryA1Red6

                )
                driveToRingPosition()
            }

            State.FOUR -> {
                //add telemetry for state (testing)
                packet.put("current state", state)
                dashboard.sendTelemetryPacket(packet)

                followTrajectories(
                    trajStorage.trajectoryC1Red1,
                    trajStorage.trajectoryA1Red2,
                    trajStorage.trajectoryA1Red3,
                    trajStorage.trajectoryA1Red4,
                    trajStorage.trajectoryC1Red5,
                    trajStorage.trajectoryA1Red6

                )
                driveToRingPosition()
            }

            State.BOUNCEBACC -> {
                if (!drive.isBusy && loopNumShooter < 3) {
                    resetDrivePosition()
                    loopNumShooter ++
                } else {
                    driveToShoot()
                }
            }

            State.RESET_DRIVE_POS -> {
                if (!drive.isBusy) {
                    driveToRingPosition()
                }
            }
            State.SHOOTER_CONTROL -> {
                //shoot, shoot, shoot
                var i = 0
                while(i < 3){
                    flickerServo.turnToAngle(1.0)
                    flickerServo.turnToAngle(0.0)
                    i++
                }

                //have shooter go down
                liftingServo1.turnToAngle(0.0)
                liftingServo2.turnToAngle(0.0)
                loopNumShooter++
            }
        }

        //write end position to class for teleOp
        PoseStorage.currentPose = drive.poseEstimate

        //sets powers (off))
        //shooterMotor.set(0.0)
        //intakeMotor.set(0.0)
    }


    companion object {
        //input the camera information, webcam info
        private const val CAMERA_WIDTH = 320 // width  of wanted camera resolution
        private const val CAMERA_HEIGHT = 240 // height of wanted camera resolution
        private const val HORIZON = 100 // horizon value to tune
        private const val DEBUG = false // if debug is wanted, change to true
        private const val USING_WEBCAM = false // change to true if using webcam
        private const val WEBCAM_NAME = "" // insert webcam name from configuration if using webcam
    }

}