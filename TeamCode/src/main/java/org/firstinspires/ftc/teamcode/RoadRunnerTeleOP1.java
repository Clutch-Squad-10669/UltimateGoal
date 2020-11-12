package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import static org.firstinspires.ftc.teamcode.RoadRunnerTeleOP1.State.*;
import static org.firstinspires.ftc.teamcode.RoadRunnerTeleOP1.State.ALIGN_TO_POINT;
import static org.firstinspires.ftc.teamcode.RoadRunnerTeleOP1.State.DRIVER_CONTROL;

/* This is the Team10669 clutch teleOP code for UG 2020-2021.
    It includes a field-relative Mecanum Drive, PID control for the linearSlide and shooterMotor, a magnetic touch sensor, an align-to-point mode, as well as a semi-autonomous mode
    The Field Relative code, augmented auto, and align-to-point mode use RoadRunner @see <a href="https://learnroadrunner.com">learnroadrunner</a>
    PID control, Motor control, SimpleServo, and Position Control use FTClib @see <a href="https://docs.ftclib.org/ftclib/">FTClib</a>
 */

@Config
@TeleOp(name="ShauryaTeleOp1")
public class RoadRunnerTeleOP1 extends LinearOpMode {

    //initialize the shooter's motor and the intake motor (ftclib)
    Motor shooterMotor = new Motor(hardwareMap, "motor1", Motor.GoBILDA.BARE);
    Motor intakeMotor = new Motor(hardwareMap, "motor2", Motor.GoBILDA.BARE);
    Motor linearSlide = new Motor(hardwareMap, "motor3", Motor.GoBILDA.RPM_1620);

    double motorVel;

    //create the servo for the wobble, one for shooter feed
    SimpleServo armServo = new SimpleServo(hardwareMap, "servo1");
    SimpleServo feedServo = new SimpleServo(hardwareMap, "servo2");

    //add the magnetic limit switch
    DigitalChannel digitalTouch;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);


    //finds the exact angle we need to turn to face the powershots
    final double anglePheta = 90 - (Math.atan((105/24)));

    //sets target radius for dashboard
    public static double DRAWING_TARGET_RADIUS = 2;


    //creates three states, driver control, align to point, and automatic control
    enum State {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL,
        ALIGN_TO_POINT
    }

    //right now we set our state to driver control
    State currentState = DRIVER_CONTROL;


    //target to align with
    private Vector2d targetPosition = new Vector2d(0, 0);

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
        motorVel = 0.0;
        shooterMotor.set(motorVel);
        intakeMotor.set(0.0);
        linearSlide.set(0.0);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //gets our pose fro the PoseStorage, which was written to at the end of the previous auto
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);


        //wait for start
        waitForStart();

        //stop if its requested
        if (isStopRequested()) return;

        shooterMotor.set(1.0);
        intakeMotor.set(1.0);
        linearSlide.set(0.0);

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
            telemetry.addData("coeffs", coeffs);
            telemetry.addData("ffcoeffs", ffCoeffs);
            telemetry.addData("slideKP1", kP1);
            telemetry.update();

             // Declare a drive direction
             // Pose representing desired x, y, and angular velocity
             Pose2d driveDirection = new Pose2d();

             telemetry.addData("mode", currentState);

             // Declare telemetry packet for dashboard field drawing
             TelemetryPacket packet = new TelemetryPacket();
             Canvas fieldOverlay = packet.fieldOverlay();

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

                    if (gamepad1.x) {
                        currentState = ALIGN_TO_POINT;
                    }

                    //right bumper is to turn on intake/shooter
                    if (gamepad2.right_bumper) {
                        motorVel = 1.0;
                        shooterMotor.set(motorVel);
                        intakeMotor.set(1.0);

                    //left bumper turns it off
                    } else if (gamepad2.left_bumper) {
                        motorVel = 0.0;
                        shooterMotor.set(motorVel);
                        intakeMotor.set(0.0);

                    //dpad up to fully extend the linear slide
                    } else if (gamepad2.x) {
                        //set position for 20cm, 0 power, 13.6 tolerance
                        linearSlide.setTargetPosition(cmToTicks(20));
                        linearSlide.set(0);
                        linearSlide.setPositionTolerance(13.6);

                        //0.75 power until at target position
                        while (!linearSlide.atTargetPosition()) {
                            linearSlide.set(0.75);
                        }

                        //stop when done
                        armServo.turnToAngle(0);//drop it off
                        linearSlide.stopMotor();

                    //dpad down to go back to where it was before
                    } else if (gamepad2.y) {
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

                        currentState = AUTOMATIC_CONTROL;


                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj2 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetBVector, targetBHeading)
                                .build();

                        drive.followTrajectoryAsync(traj2);

                        currentState = AUTOMATIC_CONTROL;


                    } else if (gamepad1.y) {

                        if (armServo.getAngle() == 0) {
                            armServo.turnToAngle(1);
                        }

                        if (armServo.getAngle() == 1) {
                            armServo.turnToAngle(0);
                        }

                    } else if (gamepad1.right_trigger > 0.5) {

                        int under = 0;

                        while (under < 3 ) {

                            feedServo.turnToAngle(1);
                            feedServo.turnToAngle(0);
                            under++;

                        }

                    } else if (gamepad1.dpad_up) {
                        motorVel += 0.05;
                        intakeMotor.set(motorVel);
                    } else if (gamepad1.dpad_down) {
                        motorVel -= 0.05;
                        intakeMotor.set(motorVel);
                    }

                    break;

                case ALIGN_TO_POINT:
                    // Switch back into normal driver control mode if `b` is pressed
                    if (gamepad1.b) {
                        currentState = DRIVER_CONTROL;
                    }

                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    // Difference between the target vector and the bot's position
                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
                    // Obtain the target angle for feedback and derivative for feedforward
                    double theta = difference.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetPosition.getX(), targetPosition.getY(), DRAWING_TARGET_RADIUS);

                    // Draw lines to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), poseEstimate.getX(), poseEstimate.getY());
                    fieldOverlay.setStroke("#ffce7a");
                    fieldOverlay.strokeLine(targetPosition.getX(), targetPosition.getY(), targetPosition.getX(), poseEstimate.getY());
                    fieldOverlay.strokeLine(targetPosition.getX(), poseEstimate.getY(), poseEstimate.getX(), poseEstimate.getY());
                    break;

                //if something happens during auto, then this breaks us out of it
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentState = DRIVER_CONTROL;
                    }

                    // If drive finishes its task, put control to the driver
                    if (!drive.isBusy()) {
                        currentState = DRIVER_CONTROL;
                        break;

                }

                // Draw bot on canvas
                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawRobot(fieldOverlay, poseEstimate);

                // Update the heading controller with our current heading
                headingController.update(poseEstimate.getHeading());

                // Update he localizer
                drive.getLocalizer().update();

                // Send telemetry packet off to dashboard
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

            }
        }
    }
}