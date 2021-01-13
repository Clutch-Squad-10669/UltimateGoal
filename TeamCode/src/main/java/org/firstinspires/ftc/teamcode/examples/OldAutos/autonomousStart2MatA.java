package org.firstinspires.ftc.teamcode.examples.OldAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.storage.PoseStorage;

public class autonomousStart2MatA extends LinearOpMode {

    //create shooterMotor and intakeMotor motor objects (bare)
    Motor shooterMotor = new Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE);
    Motor intakeMotor = new Motor(hardwareMap, "motor2", Motor.GoBILDA.BARE);

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


        Pose2d myPose = new Pose2d(-62, -25, Math.toRadians(0));

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectoryA2Red = drive.trajectoryBuilder(
                new Pose2d())
                .splineTo(new Vector2d(0, -60), Math.toRadians(0))
                .splineTo(new Vector2d(-23, -36), Math.toRadians(anglePheta))
                .splineTo(new Vector2d(10, -36), Math.toRadians(0))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        shooterMotor.set(1.0);
        intakeMotor.set(1.0);

        drive.followTrajectory(trajectoryA2Red);

        PoseStorage.currentPose = drive.getPoseEstimate();

        shooterMotor.set(0.0);
        intakeMotor.set(0.0);


    }


}
