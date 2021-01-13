package org.firstinspires.ftc.teamcode.examples.OldAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.storage.PoseStorage;


public class autonomousStart1MatA extends LinearOpMode {

    double anglePheta = 90 - (Math.atan((105 / 24)));

    Motor shooterMotor = new Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE);
    Motor intakeMotor = new Motor(hardwareMap, "motor2", Motor.GoBILDA.BARE);

    @Override
    public void runOpMode() throws InterruptedException {

        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        shooterMotor.setVeloCoefficients(0.05, 0.01, 0.31);
        double[] coeffs = shooterMotor.getVeloCoefficients();
        double kP = coeffs[0];
        double kI = coeffs[1];
        double kD = coeffs[2];

        shooterMotor.setFeedforwardCoefficients(0.92, 0.47);
        double[] ffCoeffs = shooterMotor.getFeedforwardCoefficients();
        double kS = ffCoeffs[0];
        double kV = ffCoeffs[1];


        //This tells the robot where it is on the mat to begin with
        Pose2d myPose = new Pose2d(-62, -50, Math.toRadians(0));

        //initialize hardware
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /* This is our trajectory. It says that from "myPose" we go to the C target zone,
        then we go to S position and, finally, we park on the white line. */

        //go to mat A
        Trajectory trajectoryC1Red1 = drive.trajectoryBuilder(
                new Pose2d(-62, -50, Math.toRadians(0)))
                .splineTo(new Vector2d(0, -60), Math.toRadians(0))
                //.splineTo(new Vector2d(52.0, -60.0), 0.0)
                .build();

        //pick up rings on the way to the second wobble goal
        Trajectory trajectoryC1Red2 = drive.trajectoryBuilder(
                new Pose2d(0.0, -60.0, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-20.0, -36.0, -anglePheta))
                .build();

        //continue to the second wobble goal
        Trajectory trajectoryC1Red3 = drive.trajectoryBuilder(
                new Pose2d(-20.0, -36.0, Math.toRadians(-anglePheta)))
                .lineToSplineHeading(new Pose2d(-60.0, -25.0, Math.toRadians(0)))
                .build();

        //pick up wobble and aim to powershots
        Trajectory trajectoryC1Red4 = drive.trajectoryBuilder(
                new Pose2d(-60.0, -25.0, Math.toRadians(0)))
                .splineTo(new Vector2d(-23.0, -36.0), anglePheta)
                .build();

        //Back to mat A to drop off second one
        Trajectory trajectoryC1Red5 = drive.trajectoryBuilder(
                new Pose2d(-23.0, -36.0, Math.toRadians(anglePheta)))
                //.splineTo(new Vector2d(0,-60), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(0.0, -60.0, Math.toRadians(0)))
                .build();

        //Park on line
        Trajectory trajectoryC1Red6 = drive.trajectoryBuilder(
                new Pose2d(0.0, -60.0, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(10.0, -60.0), 0.0)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        shooterMotor.set(1.0);
        intakeMotor.set(1.0);

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

        shooterMotor.set(0.0);
        intakeMotor.set(0.0);


    }


}

// General starting position would be (-62, -50)
// A configuration would be (0, -60)
// S position at (-23, -36)
// White line would be (10, -36)
