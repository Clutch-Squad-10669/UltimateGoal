package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.vision.UGContourRingPipeline
import com.arcrobotics.ftclib.vision.UGContourRingPipeline.Height
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.openftc.easyopencv.*

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

    //import trajectory storage (contains trajectory files - uses roadrunner)
    var trajStorage = TrajStorage()

    //create shooterMotor and intakeMotor motor objects (bare)
    private var shooterMotor = Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE)
    private var intakeMotor = Motor(hardwareMap, "motor2", Motor.GoBILDA.BARE)

    //initialize ftc dashboard (online driver station)
    var dashboard: FtcDashboard = FtcDashboard.getInstance()
    var packet = TelemetryPacket()

    //initialize the pipeline and camera
    private lateinit var pipeline: UGContourRingPipeline
    private lateinit var camera: OpenCvCamera
    private var cameraMonitorViewId: Int = -1

    //configure the phone camera, and webcam
    private fun configurePhoneCamera(): OpenCvInternalCamera2 = OpenCvCameraFactory.getInstance()
            .createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId
    )
    private fun configureWebCam(): OpenCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(WebcamName::class.java, WEBCAM_NAME), cameraMonitorViewId,
    )

    //create a state enum for our finite state machine
    enum class State {
        FOUR, ONE, ZERO
    }

    //start the op mode
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        //set runMode (velocity for shooter, raw for intake)
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl)
        intakeMotor.setRunMode(Motor.RunMode.RawPower)

        //set coefficients + feedforward (PID)
        shooterMotor.setVeloCoefficients(0.05, 0.01, 0.31)
        shooterMotor.setFeedforwardCoefficients(0.92, 0.47)

        //hardwareMap
        val drive = SampleMecanumDrive(hardwareMap)

        //initialize the webcam and the pipeline
        cameraMonitorViewId = hardwareMap.appContext
                .resources
                .getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.packageName,
                )

        //Configure webcam or internal camera depending on whats used
        camera = if (USING_WEBCAM) configureWebCam() else configurePhoneCamera()

        //set pipelines
        camera.setPipeline(UGContourRingPipeline(telemetry, DEBUG).apply { pipeline = this })

        //set parameters
        UGContourRingPipeline.CAMERA_WIDTH = CAMERA_WIDTH
        UGContourRingPipeline.HORIZON = HORIZON

        //start streaming to driver station
        camera.openCameraDeviceAsync {camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT)}

        //start streaming to ftc dash
        FtcDashboard.getInstance().startCameraStream(camera, 10.0)

        //set current state for zero
        var state = State.ZERO
        //get the height from the pipeline (FOUR, ONE, or ZERO), set the state depending on the height
        while (!isStarted) {
            // ^ this is very important, it makes sure that the detector is running until the opmode is started
                // the ring stack is often changed after the init is started, therefore this gives the last detected value
            state = when (pipeline.height) {
                Height.ZERO -> State.ZERO
                Height.ONE -> State.ONE
                Height.FOUR -> State.FOUR
            }
        }

        // the usual wait for start, as well as if stop requested
        waitForStart()
        if (isStopRequested) return

        //sets powers
        shooterMotor.set(1.0)
        intakeMotor.set(1.0)

        when (state) {
            State.ZERO -> {
                //add telemetry for state (testing)
                packet.put("current state", state)
                dashboard.sendTelemetryPacket(packet)
                //runTraj(TrajectoryGen1, SampleMecanumDrive())

                drive.followTrajectory(trajStorage.trajectoryA1Red1)
                drive.followTrajectory(trajStorage.trajectoryA1Red2)
                drive.followTrajectory(trajStorage.trajectoryA1Red3)
                drive.followTrajectory(trajStorage.trajectoryA1Red4)
                drive.followTrajectory(trajStorage.trajectoryA1Red5)
                drive.followTrajectory(trajStorage.trajectoryA1Red6)
            }

            State.ONE -> {
                //add telemetry for state (testing)
                packet.put("current state", state)
                dashboard.sendTelemetryPacket(packet)
                drive.followTrajectory(trajStorage.trajectoryB1Red1)
                drive.followTrajectory(trajStorage.trajectoryA1Red2)
                drive.followTrajectory(trajStorage.trajectoryA1Red3)
                drive.followTrajectory(trajStorage.trajectoryA1Red4)
                drive.followTrajectory(trajStorage.trajectoryB1Red5)
                drive.followTrajectory(trajStorage.trajectoryA1Red6)
            }

            State.FOUR -> {
                //add telemetry for state (testing)
                packet.put("current state", state)
                dashboard.sendTelemetryPacket(packet)
                drive.followTrajectory(trajStorage.trajectoryC1Red1)
                drive.followTrajectory(trajStorage.trajectoryA1Red2)
                drive.followTrajectory(trajStorage.trajectoryA1Red3)
                drive.followTrajectory(trajStorage.trajectoryA1Red4)
                drive.followTrajectory(trajStorage.trajectoryC1Red5)
                drive.followTrajectory(trajStorage.trajectoryA1Red6)
            }
        }

        //write end position to class for teleOp
        PoseStorage.currentPose = drive.poseEstimate

        //sets powers (off))
        shooterMotor.set(0.0)
        intakeMotor.set(0.0)
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