package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

public class autonomousStart1MatC extends LinearOpMode {// This line measures how much the robot has to turn to face the power shot targets.
    double anglePheta = 90 - (Math.atan((105 / 24)));

    @Override
    public void runOpMode() throws InterruptedException  {

        //This tells the robot where it is on the mat to begin with
        Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /* This is our trajectory. It says that from "myPose" we go to the C target zone,
        then we go to S position and, finally, we park on the white line. */
        Trajectory trajectoryC1Red1 = drive.trajectoryBuilder(
                new Pose2d(-62, -50, Math.toRadians(0)))
                .splineTo(new Vector2d(52.0, -60.0), 0.0)
                .build();

        Trajectory trajectoryC1Red2 = drive.trajectoryBuilder(
                new Pose2d(52.0, -60.0,  Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-20.0, -36.0, -anglePheta))
                .build();

        Trajectory trajectoryC1Red3 = drive.trajectoryBuilder(
                new Pose2d(-20.0, -36.0,  Math.toRadians(-anglePheta)))
                .lineToSplineHeading(new Pose2d(-60.0, -25.0, Math.toRadians(0)))
                .build();

        Trajectory trajectoryC1Red4 = drive.trajectoryBuilder(
                new Pose2d(-60.0, -25.0,  Math.toRadians(0)))
                .splineTo(new Vector2d(-23.0, -36.0), anglePheta)
                .build();

        Trajectory trajectoryC1Red5 = drive.trajectoryBuilder(
                new Pose2d(-23.0, -36.0,  Math.toRadians(anglePheta)))
                .lineToSplineHeading(new Pose2d(54.0, -60.0, Math.toRadians(0)))
                .build();

        Trajectory trajectoryC1Red6 = drive.trajectoryBuilder(
                new Pose2d(54.0, -60.0,  Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(10.0, -60.0), 0.0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //This simply tells the robot to follow the trajectory above
        drive.followTrajectory(trajectoryC1Red1);
        drive.followTrajectory(trajectoryC1Red2);
        drive.followTrajectory(trajectoryC1Red3);
        drive.followTrajectory(trajectoryC1Red4);
        drive.followTrajectory(trajectoryC1Red5);
        drive.followTrajectory(trajectoryC1Red6);

        /* This saves the coordinates in another program so that the robot can recognize where the coordinates
        are when it sees them again */
        PoseStorage.currentPose = drive.getPoseEstimate();


    }
}
