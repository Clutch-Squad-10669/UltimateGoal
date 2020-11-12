package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

/* This is the Team10669 clutch teleOP code for UG 2020-2021.
    It includes a field-relative Mecanum Drive, PID control for the linearSlide and shooterMotor, a magnetic touch sensor, as well as a semi-autonomous mode
    The Field Relative code uses RoadRunner @see <a href="https://learnroadrunner.com">learnroadrunner</a>
    PID control, Motor control, SimpleServo, and Position Control use FTClib @see <a href="https://docs.ftclib.org/ftclib/">FTClib</a>
 */

@Config
@TeleOp(name="ShauryaTeleOp1")
public class RoadRunnerTeleOP1 extends LinearOpMode {

    //initialize the shooter's motor and the intake motor (ftclib)
    Motor shooterMotor = new Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE);
    Motor intakeMotor = new Motor(hardwareMap, "motor2", Motor.GoBILDA.BARE);
    Motor linearSlide = new Motor(hardwareMap, "motor3", Motor.GoBILDA.RPM_1620);

    //add the magnetic limit switch
    DigitalChannel digitalTouch;

    //finds the exact angle we need to turn to face the powershots
    final double anglePheta = 90 - (Math.atan((105/24)));

    //creates two states, driver control and automatic control
    enum State {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    //right now we set our state to driver control
    State currentState = State.DRIVER_CONTROL;

    //we set the target vector to -23, -26 which are the coordinates of the ring
    Vector2d targetAVector = new Vector2d(-23, -36);
    Vector2d targetBVector = new Vector2d(-23, -36);

    //we set the heading to anglePheta, which is around 12.88
    double targetAHeading = Math.toRadians(anglePheta);
    double targetBHeading = Math.toRadians(anglePheta + 10);

    //name gear ratio wheel radius and ticks for encoder cm conversion
    public static double GEAR_RATIO = 1.0; // for simulator
    public static double WHEEL_RADIUS = 1.0;  // 5 cm
    public static double TICKS_PER_ROTATION = 103.6;  // From NeveRest (for simulator)

    //calculate cm
    double CM_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS) / TICKS_PER_ROTATION;

    //calculate ticks from cm, return ticks
    private int cmToTicks(double cm) {
        double cmTick = (cm/CM_PER_TICK);
        return (int) cmTick;
    }

    //declare @override
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        //we use PID to maintain a constant velocity on the shooter, and normally drive the intake (this uses FTClib)
        //linearSlide is set using runToPosition
        //initialize runmode
        shooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        linearSlide.setRunMode(Motor.RunMode.PositionControl);

            //sets coeffs for PID motor 1
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

            linearSlide.setPositionCoefficient(0.05);
            double kP1 = linearSlide.getPositionCoefficient();

        //sets powers (temporary)
        shooterMotor.set(1.0);
        intakeMotor.set(1.0);
        linearSlide.set(0.0);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //gets our pose fro the PoseStorage, which was written to at the end of the previous auto
        drive.setPoseEstimate(PoseStorage.currentPose);

        //wait for start
        waitForStart();

        //stop if its requested
        if (isStopRequested()) return;

        //start the loop, manually
         while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch control to automatic
            switch (currentState) { //tells it based on what state it is to do something
                //this is for driver control
                case DRIVER_CONTROL:
                    // Translate gamepad inputs into velocity
                    //account current heading into vector
                    Vector2d input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());

                    //correct using PID, then put the target heading into the pose
                    drive.setWeightedDrivePower(
                        new Pose2d(
                                input.getX(),
                                input.getY(),
                                -gamepad1.right_stick_x
                        )
                    );

                    //right bumper is to turn on intake/shooter
                    if (gamepad1.right_bumper) {
                        shooterMotor.set(1.0);
                        intakeMotor.set(1.0);

                    //left bumper turns it off
                    } else if (gamepad1.left_bumper) {
                        shooterMotor.set(0.0);
                        intakeMotor.set(0.0);

                    //dpad up to fully extend the linear slide
                    } else if (gamepad1.dpad_up) {
                        //set position for 20cm, 0 power, 13.6 tolerance
                        linearSlide.setTargetPosition(cmToTicks(20));
                        linearSlide.set(0);
                        linearSlide.setPositionTolerance(13.6);

                        //0.75 power until at target position
                        while (!linearSlide.atTargetPosition()) {
                            linearSlide.set(0.75);
                        }

                        //stop when done
                        linearSlide.stopMotor();

                    //dpad down to go back to where it was before
                    } else if (gamepad1.dpad_down) {
                        //set position for -20cm (relative), 0 power, 13.6 tolerence
                        linearSlide.setTargetPosition(cmToTicks(0));
                        linearSlide.set(0);
                        linearSlide.setPositionTolerance(13.6);

                        //continue until limit switch state is true (detects slide)
                        while (!digitalTouch.getState()) {
                            linearSlide.set(0.75);
                        }

                        //stop motor
                        linearSlide.stopMotor();

                    } else if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentState = State.AUTOMATIC_CONTROL;

                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj2 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetBVector, targetBHeading)
                                .build();

                        drive.followTrajectoryAsync(traj2);

                        currentState = State.AUTOMATIC_CONTROL;
                    }

                    break;

                //if something happens during auto, then this breaks us out of it
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentState = State.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, put control to the driver
                    if (!drive.isBusy()) {
                        currentState = State.DRIVER_CONTROL;
                        break;

                }
            }
        }
    }
}


