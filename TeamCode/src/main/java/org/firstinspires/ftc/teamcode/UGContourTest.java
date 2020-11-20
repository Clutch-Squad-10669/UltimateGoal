package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

import static com.arcrobotics.ftclib.vision.UGContourRingPipeline.*;
import static com.arcrobotics.ftclib.vision.UGContourRingPipeline.Height.*;

@Autonomous(name = "UGAutoTest")
public class UGContourTest extends LinearOpMode {

    private static final int CAMERA_WIDTH = 320; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 240; // height of wanted camera resolution

    private static final int HORIZON = 100; // horizon value to tune

    private static final boolean DEBUG = false; // if debug is wanted, change to true

    private static final boolean USING_WEBCAM = false; // change to true if using webcam
    private static final String WEBCAM_NAME = ""; // insert webcam name from configuration if using webcam

    FtcDashboard dashboard = FtcDashboard.getInstance();
    //Telemetry dashboardTelemetry = dashboard.getTelemetry();
    TelemetryPacket packet = new TelemetryPacket();
    //FtcDashboard.getInstance().startCameraStream(phoneCam, 30);

    private UGContourRingPipeline pipeline;
    private OpenCvCamera camera;


    enum State {
        FOUR,
        ONE,
        ZERO
    }

    @Override
    public void runOpMode() throws InterruptedException {
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

        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, DEBUG));

        Config.setCAMERA_WIDTH(CAMERA_WIDTH);

        Config.setHORIZON(HORIZON);

        //camera.openCameraDeviceAsync(() -> camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT));

        FtcDashboard.getInstance().startCameraStream(camera, 0);

        State state = State.ZERO;

        while (!isStarted()) {
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


        waitForStart();

        if (isStopRequested()) return;

        switch (state) {
            case ZERO:
                break;
            case ONE:
                break;
            case FOUR:
                break;

        }

        
    }

}