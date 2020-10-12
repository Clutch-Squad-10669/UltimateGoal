package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@Autonomous (name = "RoadRunnerAutonomous1")
public class RoadRunnerAutonomous1 extends LinearOpMode
{

    private DcMotor shooter;

    double setMotorTime = 1; // What time we set the motor power
    double setMotorWait = 1.5; // How long we wait until we turn off the motor

    double anglePheta = 90 - (Math.atan((105/24)));

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectoryA1Red = drive.trajectoryBuilder(
                new Pose2d())
                .splineTo(new Vector2d(0,-60), Math.toRadians(0))
                .splineTo(new Vector2d(-23,-36), Math.toRadians(anglePheta))
                .splineTo(new Vector2d(10,-36), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(trajectoryA1Red);

        PoseStorage.currentPose = drive.getPoseEstimate();


    }


}

// General starting position would be (-62, -50)
// A configuration would be (0, -60)
// S position at (-23, -36)
// White line would be (10, -36)
