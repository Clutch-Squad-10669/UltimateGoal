package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

public class autonomousStart1MatB extends LinearOpMode {

    //initialize the shooter's motor and the intake motor
    Motor shooterMotor = new Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE);
    Motor intakeMotor = new Motor(hardwareMap,"motor2", Motor.GoBILDA.BARE);

    // This is the angle that the robot has to turn to face the target from the point where the rings are
    double anglePheta = 90 - (Math.atan((105 / 24)));

    // This is our main method

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d myPose = new Pose2d(-62, -25, Math.toRadians(0));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectoryB2Red = drive.trajectoryBuilder(
                new Pose2d())
                .splineTo(new Vector2d(34, -35), Math.toRadians(0))
                .splineTo(new Vector2d(-23, -36), Math.toRadians(anglePheta))
                .splineTo(new Vector2d(10, -36), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectoryB2Red);

        PoseStorage.currentPose = drive.getPoseEstimate();

    }
}
