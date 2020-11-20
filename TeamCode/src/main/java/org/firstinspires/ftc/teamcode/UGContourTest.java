package org.firstinspires.ftc.teamcode;

//general imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//eocv imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

import static com.arcrobotics.ftclib.vision.UGContourRingPipeline.*;

//declare the autonomous
@Autonomous(name = "UGAutoTest")
public class UGContourTest extends LinearOpMode {

    //create objects of our other autonomous OpModes to make cleaner code
    autonomousStart1MatA start1MatA = new autonomousStart1MatA();
    autonomousStart1MatB start1MatB = new autonomousStart1MatB();
    autonomousStart1MatC start1MatC = new autonomousStart1MatC();

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
        //camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        //start streaming to ftc dash
        FtcDashboard.getInstance().startCameraStream(camera, 0);

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

        //we are going to run a specific op mode depending on the state detected
        switch (state) {
            case ZERO:
                start1MatA.runOpMode();
                break;
            case ONE:
                start1MatB.runOpMode();
                break;
            case FOUR:
                start1MatC.runOpMode();
                break;
        }

        
    }

}