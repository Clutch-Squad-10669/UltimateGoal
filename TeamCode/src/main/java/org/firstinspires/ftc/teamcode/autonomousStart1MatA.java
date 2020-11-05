package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@Autonomous (name = "RoadRunnerAutonomous1A")
public class autonomousStart1MatA extends LinearOpMode
{
        double anglePheta = 90 - (Math.atan((105 / 24)));

        @Override
        public void runOpMode() throws InterruptedException  {

            //This tells the robot where it is on the mat to begin with
            Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));

            //initialize hardware
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /* This is our trajectory. It says that from "myPose" we go to the C target zone,
        then we go to S position and, finally, we park on the white line. */

            //go to mat A
            Trajectory trajectoryC1Red1 = drive.trajectoryBuilder(
                    new Pose2d(-62, -50, Math.toRadians(0)))
                    .splineTo(new Vector2d(0,-60), Math.toRadians(0))
                    //.splineTo(new Vector2d(52.0, -60.0), 0.0)
                    .build();

            //pick up rings on the way to the second wobble goal
            Trajectory trajectoryC1Red2 = drive.trajectoryBuilder(
                    new Pose2d(0.0, -60.0,  Math.toRadians(0)))
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

            //Back to mat A to drop off second one
            Trajectory trajectoryC1Red5 = drive.trajectoryBuilder(
                    new Pose2d(-23.0, -36.0,  Math.toRadians(anglePheta)))
                    //.splineTo(new Vector2d(0,-60), Math.toRadians(0))
                    .lineToSplineHeading(new Pose2d(0.0, -60.0, Math.toRadians(0)))
                    .build();

            //Park on line
            Trajectory trajectoryC1Red6 = drive.trajectoryBuilder(
                    new Pose2d(0.0, -60.0,  Math.toRadians(0)))
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

// General starting position would be (-62, -50)
// A configuration would be (0, -60)
// S position at (-23, -36)
// White line would be (10, -36)
