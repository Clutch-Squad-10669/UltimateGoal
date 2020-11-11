package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

@Autonomous(name = "RoadRunnerAutonomous1B")
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

        //intialize RunMode
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

            //set coefficients for PID motor 1
            shooterMotor.setVeloCoefficients(0.05, 0.01, 0.31);
            double[] coeffs = shooterMotor.getVeloCoefficients();
            double kP = coeffs[0];
            double kI = coeffs[1];
            double kD = coeffs[2];

            //set and get the feedforwards coefficients
            shooterMotor.setFeedforwardCoefficients(0.92, 0.47);
            double[] ffCoeffs = shooterMotor.getFeedforwardCoefficients();
            double kS = ffCoeffs[0];
            double kV = ffCoeffs[1];


        waitForStart();

        if (isStopRequested()) return;

        //sets powers (temporary)
        shooterMotor.set(1.0);
        intakeMotor.set(1.0);

        drive.followTrajectory(trajectoryB2Red);

        PoseStorage.currentPose = drive.getPoseEstimate();

        //sets powers (off))
        shooterMotor.set(0.0);
        intakeMotor.set(0.0);


    }
}
