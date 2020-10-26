package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.EasyOpenCVDedector;
import org.firstinspires.ftc.teamcode.EasyOpenCVExample;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="AutoTest1")
public class openCVAutoTest extends LinearOpMode {

    OpenCvInternalCamera phoneCam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    EasyOpenCVDedector RingStack = new EasyOpenCVDedector();

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        RingStack.runOpMode();

    }
}
