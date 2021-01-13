package org.firstinspires.ftc.teamcode.examples.OldAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.storage.PoseStorage;


public class autonomousStart1MatB extends LinearOpMode {

    //create shooterMotor and intakeMotor motor objects (bare)
    Motor shooterMotor = new Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE);
    Motor intakeMotor = new Motor(hardwareMap, "motor2", Motor.GoBILDA.BARE);

    //create the servo for the wobble
    SimpleServo armServo = new SimpleServo(hardwareMap, "servo1");

    double anglePheta = 90 - (Math.atan((105 / 24)));

    @Override
    public void runOpMode() throws InterruptedException {

        //set runMode (velo for shooter, raw for intake)
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        //set coeffs
        shooterMotor.setVeloCoefficients(0.05, 0.01, 0.31);
        double[] coeffs = shooterMotor.getVeloCoefficients();
        double kP = coeffs[0];
        double kI = coeffs[1];
        double kD = coeffs[2];

        // set and get the feedforward coefficients
        shooterMotor.setFeedforwardCoefficients(0.92, 0.47);
        double[] ffCoeffs = shooterMotor.getFeedforwardCoefficients();
        double kS = ffCoeffs[0];
        double kV = ffCoeffs[1];

        //This tells the robot where it is on the mat to begin with
        Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));

        //initialize hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /* This is our trajectory. It says that from "myPose" we go to the B target zone,
        then we go to S position and, finally, we park on the white line. */

        //go to mat B
        Trajectory trajectoryB1Red1 = drive.trajectoryBuilder(
                new Pose2d(-62, -50, Math.toRadians(0)))
                .splineTo(new Vector2d(34.0, -35.0), 0.0)
                .build();

        //pick up rings on the way to the second wobble goal
        Trajectory trajectoryB1Red2 = drive.trajectoryBuilder(
                new Pose2d(34.0, -35.0, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-20.0, -36.0, -anglePheta))
                .build();

        //continue to the second wobble goal
        Trajectory trajectoryB1Red3 = drive.trajectoryBuilder(
                new Pose2d(-20.0, -36.0, Math.toRadians(-anglePheta)))
                .lineToSplineHeading(new Pose2d(-60.0, -25.0, Math.toRadians(0)))
                .build();

        //pick up wobble and aim to powershots
        Trajectory trajectoryB1Red4 = drive.trajectoryBuilder(
                new Pose2d(-60.0, -25.0, Math.toRadians(0)))
                .splineTo(new Vector2d(-23.0, -36.0), anglePheta)
                .build();

        //Back to mat B to drop off second one
        Trajectory trajectoryB1Red5 = drive.trajectoryBuilder(
                new Pose2d(-23.0, -36.0, Math.toRadians(anglePheta)))
                .lineToSplineHeading(new Pose2d(34.0, -35.0, Math.toRadians(0)))
                .build();

        //Park on line
        Trajectory trajectoryB1Red6 = drive.trajectoryBuilder(
                new Pose2d(34.0, -35.0, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(10.0, -60.0), 0.0)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        //sets powers
        shooterMotor.set(1.0);
        intakeMotor.set(1.0);

        //This simply tells the robot to follow the trajectory above

        armServo.turnToAngle(1);//pick it up
        //top right
        drive.followTrajectory(trajectoryB1Red1);
        armServo.turnToAngle(0);//drop it off
        //pick up rings
        drive.followTrajectory(trajectoryB1Red2);
        //continue
        drive.followTrajectory(trajectoryB1Red3);
        //pick up second wobble goal, to powershots
        drive.followTrajectory(trajectoryB1Red4);
        armServo.turnToAngle(1);//pick it up
        //back to mat B drop it off
        drive.followTrajectory(trajectoryB1Red5);
        armServo.turnToAngle(0);//drop it off
        //park on line
        drive.followTrajectory(trajectoryB1Red6);

        /* This saves the coordinates in another program so that the robot can recognize where the coordinates
        are when it sees them again */
        PoseStorage.currentPose = drive.getPoseEstimate();

        //sets powers (off))
        shooterMotor.set(0.0);
        intakeMotor.set(0.0);


    }
}
