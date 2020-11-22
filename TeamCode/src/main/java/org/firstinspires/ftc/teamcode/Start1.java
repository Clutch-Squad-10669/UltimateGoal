package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EOCVtests.UGContourTest;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import static com.arcrobotics.ftclib.vision.UGContourRingPipeline.Config;

@Autonomous(name = "Start1Auto")
public class Start1 extends LinearOpMode {

    //create objects of our other autonomous OpModes to make cleaner code
    //autonomousStart1MatA start1MatA = new autonomousStart1MatA();
    //autonomousStart1MatB start1MatB = new autonomousStart1MatB();
    //autonomousStart1MatC start1MatC = new autonomousStart1MatC();

    //create shooterMotor and intakeMotor motor objects (bare)
    Motor shooterMotor = new Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE);
    Motor intakeMotor = new Motor(hardwareMap, "motor2", Motor.GoBILDA.BARE);

    //create the servo for the wobble
    SimpleServo armServo = new SimpleServo(hardwareMap, "servo1");

    //angle pheta and general other imports
    double anglePheta = 90 - (Math.atan((105 / 24)));

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
    enum State {
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

        //set coeffs
        shooterMotor.setVeloCoefficients(0.05, 0.01, 0.31);
        // set and get the feedforward coefficients
        shooterMotor.setFeedforwardCoefficients(0.92, 0.47);

        //This tells the robot where it is on the mat to begin with
        Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));

        //initialize hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /* This is our trajectory. It says that from "myPose" we go to the C target zone,
        then we go to S position and, finally, we park on the white line. */

        //go to mat C
        Trajectory trajectoryC1Red1 = drive.trajectoryBuilder(
                new Pose2d(-62, -50, Math.toRadians(0)))
                .splineTo(new Vector2d(52.0, -60.0), 0.0)
                .build();

        //start a
        Trajectory trajectoryC2Red1 = drive.trajectoryBuilder(
                new Pose2d(-62, -50, Math.toRadians(0)))
                .splineTo(new Vector2d(0,-60), Math.toRadians(0))
                //.splineTo(new Vector2d(52.0, -60.0), 0.0)
                .build();

        //pick up rings on the way to the second wobble goal
        Trajectory trajectoryC1Red2 = drive.trajectoryBuilder(
                new Pose2d(52.0, -60.0,  Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-20.0, -36.0, -anglePheta))
                .build();

        //continue to the second wobble goal
        Trajectory trajectoryC1Red3 = drive.trajectoryBuilder(
                new Pose2d(-20.0, -36.0,  Math.toRadians(-anglePheta)))
                .lineToSplineHeading(new Pose2d(-60.0, -25.0, Math.toRadians(0)))
                .build();

        //pick up wobble and aim to powershots
        Trajectory trajectoryC1Red4 = drive.trajectoryBuilder(
                new Pose2d(-60.0, -25.0,  Math.toRadians(0)))
                .splineTo(new Vector2d(-23.0, -36.0), anglePheta)
                .build();

        //Back to mat C to drop off second one
        Trajectory trajectoryC1Red5 = drive.trajectoryBuilder(
                new Pose2d(-23.0, -36.0,  Math.toRadians(anglePheta)))
                .lineToSplineHeading(new Pose2d(54.0, -60.0, Math.toRadians(0)))
                .build();

        //Back to mat A to drop off second one
        Trajectory trajectoryC2Red5 = drive.trajectoryBuilder(
                new Pose2d(-23.0, -36.0,  Math.toRadians(anglePheta)))
                //.splineTo(new Vector2d(0,-60), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(0.0, -60.0, Math.toRadians(0)))
                .build();

        //Park on line
        Trajectory trajectoryC1Red6 = drive.trajectoryBuilder(
                new Pose2d(54.0, -60.0,  Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(10.0, -60.0), 0.0)
                .build();


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
        UGContourTest.State state = UGContourTest.State.ZERO;

        //get the height from the pipeline (FOUR, ONE, or ZERO)
        //set the state depending on the height

        while (!isStarted()) {
            // ^ this is very important, it makes sure that the detector is running until the opmode is started
            //the ring stack is often changed after the init is started, therefore this gives the last detected value
            switch (pipeline.getHeight()) {
                case ZERO:
                    state = UGContourTest.State.ZERO;
                    break;
                case ONE:
                    state = UGContourTest.State.ONE;
                    break;
                case FOUR:
                    state = UGContourTest.State.FOUR;
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

                drive.followTrajectory(trajectoryC2Red1);
                drive.followTrajectory(trajectoryC1Red2);
                drive.followTrajectory(trajectoryC1Red3);
                drive.followTrajectory(trajectoryC1Red4);
                drive.followTrajectory(trajectoryC2Red5);
                drive.followTrajectory(trajectoryC1Red6);

                break;

            case ONE:
                //add telemetry for state (testing)
                packet.put("current state", state);
                dashboard.sendTelemetryPacket(packet);

                break;

            case FOUR:
                //add telemetry for state (testing)
                packet.put("current state", state);
                dashboard.sendTelemetryPacket(packet);

                drive.followTrajectory(trajectoryC1Red1);
                drive.followTrajectory(trajectoryC1Red2);
                drive.followTrajectory(trajectoryC1Red3);
                drive.followTrajectory(trajectoryC1Red4);
                drive.followTrajectory(trajectoryC1Red5);
                drive.followTrajectory(trajectoryC1Red6);

                break;

        }

        PoseStorage.currentPose = drive.getPoseEstimate();

        //sets powers (off))
        shooterMotor.set(0.0);
        intakeMotor.set(0.0);


    }

}