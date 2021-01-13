package org.firstinspires.ftc.teamcode.examples.OldAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.storage.PoseStorage;

public class autonomousStart4MatB extends LinearOpMode {

    double anglePheta = 90 - (Math.atan((105 / 24)));

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d myPose = new Pose2d(-62, 50, Math.toRadians(0));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectoryB4Blue = drive.trajectoryBuilder(
                new Pose2d())
                .splineTo(new Vector2d(34, 35), Math.toRadians(0))
                .splineTo(new Vector2d(-23, 36), Math.toRadians(-anglePheta))
                .splineTo(new Vector2d(10, 36), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectoryB4Blue);

        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}
