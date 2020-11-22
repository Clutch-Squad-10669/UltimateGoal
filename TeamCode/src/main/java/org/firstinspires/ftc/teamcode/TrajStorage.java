package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class TrajStorage {

    double anglePheta = 90 - (Math.atan((105 / 24)));

    Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //go to mat A
    Trajectory trajectoryA1Red1 = drive.trajectoryBuilder(
            new Pose2d(-62, -50, Math.toRadians(0)))
            .splineTo(new Vector2d(0,-60), Math.toRadians(0))
            //.splineTo(new Vector2d(52.0, -60.0), 0.0)
            .build();

    //pick up rings on the way to the second wobble goal
    Trajectory trajectoryA1Red2 = drive.trajectoryBuilder(
            new Pose2d(0.0, -60.0,  Math.toRadians(0)))
            .lineToSplineHeading(new Pose2d(-20.0, -36.0, -anglePheta))
            .build();

    //continue to the second wobble goal
    Trajectory trajectoryA1Red3 = drive.trajectoryBuilder(
            new Pose2d(-20.0, -36.0,  Math.toRadians(-anglePheta)))
            .lineToSplineHeading(new Pose2d(-60.0, -25.0, Math.toRadians(0)))
            .build();

    //pick up wobble and aim to powershots
    Trajectory trajectoryA1Red4 = drive.trajectoryBuilder(
            new Pose2d(-60.0, -25.0,  Math.toRadians(0)))
            .splineTo(new Vector2d(-23.0, -36.0), anglePheta)
            .build();

    //Back to mat A to drop off second one
    Trajectory trajectoryA1Red5 = drive.trajectoryBuilder(
            new Pose2d(-23.0, -36.0,  Math.toRadians(anglePheta)))
            //.splineTo(new Vector2d(0,-60), Math.toRadians(0))
            .lineToSplineHeading(new Pose2d(0.0, -60.0, Math.toRadians(0)))
            .build();

    //Park on line
    Trajectory trajectoryA1Red6 = drive.trajectoryBuilder(
            new Pose2d(0.0, -60.0,  Math.toRadians(0)))
            .splineToConstantHeading(new Vector2d(10.0, -60.0), 0.0)
            .build();

    //go to mat B
    Trajectory trajectoryB1Red1 = drive.trajectoryBuilder(
            new Pose2d(-62, -50, Math.toRadians(0)))
            .splineTo(new Vector2d(34.0, -35.0), 0.0)
            .build();

    //Back to mat B to drop off second one
    Trajectory trajectoryB1Red5 = drive.trajectoryBuilder(
            new Pose2d(-23.0, -36.0,  Math.toRadians(anglePheta)))
            .lineToSplineHeading(new Pose2d(34.0, -35.0, Math.toRadians(0)))
            .build();

    //go to mat C
    Trajectory trajectoryC1Red1 = drive.trajectoryBuilder(
            new Pose2d(-62, -50, Math.toRadians(0)))
            .splineTo(new Vector2d(52.0, -60.0), 0.0)
            .build();

    //Back to mat C to drop off second one
    Trajectory trajectoryC1Red5 = drive.trajectoryBuilder(
            new Pose2d(-23.0, -36.0,  Math.toRadians(anglePheta)))
            .lineToSplineHeading(new Pose2d(54.0, -60.0, Math.toRadians(0)))
            .build();

}
