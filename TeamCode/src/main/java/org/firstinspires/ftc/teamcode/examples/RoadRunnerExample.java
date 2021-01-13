package org.firstinspires.ftc.teamcode.examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class RoadRunnerExample extends LinearOpMode {

    double anglePheta = 90 - (Math.atan((105 / 24)));

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(
                //new Pose2d myPose = myLocalizer.getPoseEstimate());
                new Pose2d())
                .splineTo(new Vector2d(-23, -36), Math.toRadians(anglePheta))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectory(myTrajectory);

    }

}