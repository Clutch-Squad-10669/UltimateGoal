package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import static com.arcrobotics.ftclib.vision.UGContourRingPipeline.Config;

/*
    This is the Team10669 clutch teleOP code for UG 2020-2021.
    It includes a contour-based ring detector, finite state machines, and PID control for various motors
    Odometry is done through the roadrunner library @see <a href="https://learnroadrunner.com">learnroadrunner</a>
    The vision pipeline, servo control, and PID control is done through FTClib  @see <a href="https://docs.ftclib.org/ftclib/">FTClib</a>
 */

//declare autonomous
@Autonomous(name = "Start1Auto")
public class Start1 extends LinearOpMode {

    //import traj storage (contains trajectory files - uses roadrunner)
    TrajStorage trajStorage = new TrajStorage();

    //create shooterMotor and intakeMotor motor objects (bare)
    Motor shooterMotor = new Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE);
    Motor intakeMotor = new Motor(hardwareMap, "motor2", Motor.GoBILDA.BARE);

    //input the camera information, webcam info
    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution
    private static final int HORIZON = 100; // horizon value to tune
    private static final boolean DEBUG = false; // if debug is wanted, change to true
    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    //initialize ftc dashboard (online driver station)
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    //initialize the pipeline and camera
    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;

    //create a state enum for our finite state machine
    public enum State {
        FOUR,
        ONE,
        ZERO,
        FINAL
    }

    //start the op mode
    @Override
    public void runOpMode() throws InterruptedException {

        //set runMode (velo for shooter, raw for intake)
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        //set coeffs + feedforward (PID)
        shooterMotor.setVeloCoefficients(0.05, 0.01, 0.31);
        shooterMotor.setFeedforwardCoefficients(0.92, 0.47);

        //hardwareMap
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //initialize the webcam and the pipeline
        int cameraMonitorViewId = this
                .hardwareMap
                .appContext
                .getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );
        if (USING_WEBCAM) {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        } else {
            camera = OpenCvCameraFactory
                    .getInstance()
                    .createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        }

        //set pipelines
        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        //set paramters
        Config.setCAMERA_WIDTH(CAMERA_WIDTH);
        Config.setHORIZON(HORIZON);

        //start streaming to driverstation
        camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        //start streaming to ftc dash
        FtcDashboard.getInstance().startCameraStream(camera, 10);

        //set current state for zero
        State state = State.ZERO;

        //get the height from the pipeline (FOUR, ONE, or ZERO)
        //set the state depending on the height

        while (!isStarted()) {
            // ^ this is very important, it makes sure that the detector is running until the opmode is started
            //the ring stack is often changed after the init is started, therefore this gives the last detected value
            switch (pipeline.getHeight()) {
                case ZERO:
                    state = State.ZERO;
                    break;
                case ONE:
                    state = State.ONE;
                    break;
                case FOUR:
                    state = State.FOUR;
                    break;
            }

        }

        // the usual wait for start, as well as if stop requested
        waitForStart();
        if (isStopRequested()) return;

        //sets powers
        shooterMotor.set(1.0);
        intakeMotor.set(1.0);

        //we are going to run a specific op mode depending on the state detected
        switch (state) {
            case ZERO:
                //add telemetry for state (testing)
                packet.put("current state", state);
                dashboard.sendTelemetryPacket(packet);

                drive.followTrajectory(trajStorage.trajectoryA1Red1);
                drive.followTrajectory(trajStorage.trajectoryA1Red2);
                drive.followTrajectory(trajStorage.trajectoryA1Red3);
                drive.followTrajectory(trajStorage.trajectoryA1Red4);
                drive.followTrajectory(trajStorage.trajectoryA1Red5);
                drive.followTrajectory(trajStorage.trajectoryA1Red6);

                break;

            case ONE:
                //add telemetry for state (testing)
                packet.put("current state", state);
                dashboard.sendTelemetryPacket(packet);

                drive.followTrajectory(trajStorage.trajectoryB1Red1);
                drive.followTrajectory(trajStorage.trajectoryA1Red2);
                drive.followTrajectory(trajStorage.trajectoryA1Red3);
                drive.followTrajectory(trajStorage.trajectoryA1Red4);
                drive.followTrajectory(trajStorage.trajectoryB1Red5);
                drive.followTrajectory(trajStorage.trajectoryA1Red6);

                break;

            case FOUR:
                //add telemetry for state (testing)
                packet.put("current state", state);
                dashboard.sendTelemetryPacket(packet);

                drive.followTrajectory(trajStorage.trajectoryC1Red1);
                drive.followTrajectory(trajStorage.trajectoryA1Red2);
                drive.followTrajectory(trajStorage.trajectoryA1Red3);
                drive.followTrajectory(trajStorage.trajectoryA1Red4);
                drive.followTrajectory(trajStorage.trajectoryC1Red5);
                drive.followTrajectory(trajStorage.trajectoryA1Red6);

                break;

        }

        //write end position to class for teleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        //sets powers (off))
        shooterMotor.set(0.0);
        intakeMotor.set(0.0);


    }

}