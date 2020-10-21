package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@Autonomous (name = "RoadRunnerAutonomous1")
public class autonomousStart1MatA extends LinearOpMode
{
    /*
    calculates the angle the robot needs to turn to face the Power Shot targets
    from Position S, the robot would need to turn a certain amount of degrees to face the Power Shot target
    using trigonometry, we were able to find that angle
    wherever the robot is facing, after calling anglePheta, the robot will turn to face the targets
     */
    double anglePheta = 90 - (Math.atan((105/24)));

    @Override
    public void runOpMode() throws InterruptedException {
        //the starting position of the robot: S Position
        Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));
        //gets SampleMacanumDrive from the hardwardMap
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //new trajectory: goes to MatA, goes to S Position, and ends on the white line
        Trajectory trajectoryA1Red = drive.trajectoryBuilder(
                new Pose2d())
                .splineTo(new Vector2d(0,-60), Math.toRadians(0))
                .splineTo(new Vector2d(-23,-36), Math.toRadians(anglePheta))
                .splineTo(new Vector2d(10,-36), Math.toRadians(0))
                .build();
        //wait for start
        waitForStart();

        if(isStopRequested()) return;

        //tells robot to follow trajectory
        drive.followTrajectory(trajectoryA1Red);

        /*This saves the coordinates in another program so that the robot can recognize where the coordinates
        are whes the coordinates again */
        PoseStorage.currentPose = drive.getPoseEstimate();


    }


}

// General starting position would be (-62, -50)
// A configuration would be (0, -60)
// S position at (-23, -36)
// White line would be (10, -36)
