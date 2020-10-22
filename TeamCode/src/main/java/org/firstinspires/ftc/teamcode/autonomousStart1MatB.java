package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

public class autonomousStart1MatB extends LinearOpMode {

    // This is the angle that the robot has to turn to face the target from the point where the rings are
    double anglePheta = 90 - (Math.atan((105 / 24)));

    // This is our main method

    @Override
    public void runOpMode() throws InterruptedException {

        // Here, we are creating a pose. The starting position is (-62, -50).
        Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // This is the beginning of our trajectory
        Trajectory trajectoryB1Red = drive.trajectoryBuilder(
                new Pose2d())
                // These are the codes that tell the robot to spline to a certain point and specific direction
                .splineTo(new Vector2d(34, -35), Math.toRadians(0))
                .splineTo(new Vector2d(-23, -36), Math.toRadians(anglePheta))
                .splineTo(new Vector2d(10, -36), Math.toRadians(0))
                .build();

        // These are some lines of code that tell the robot to follow our trajectory and perform subsequent activities

        waitForStart();

        if (isStopRequested()) return;

        // This simply tells the robot to follow the trajectory above
        drive.followTrajectory(trajectoryB1Red);

        /* This saves the coordinates in another program so that the robot can recognize where the coordinates
        it sees them again */
        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}
