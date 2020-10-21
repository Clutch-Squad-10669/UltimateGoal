package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

@Config
@TeleOp(name="ShauryaTeleOp1")
public class RoadRunnerTeleOP1 extends LinearOpMode {

    pointCenteredTeleOP pointcentered = new pointCenteredTeleOP();

    //finds the exact angle we need to turn to face the powershots
    final double anglePheta = 90 - (Math.atan((105/24)));

    //creates two states, driver control and automatic control
    enum State {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL;
    }

    //right now we set our state to driver control
    State currentState = State.DRIVER_CONTROL;

    //we set the target vector to -23, -26 which are the coordinates of the ring
    Vector2d targetAVector = new Vector2d(-23, -36);

    //we set the heading to anglePheta, which is around 12.88
    double targetAHeading = Math.toRadians(anglePheta);

    //declare @override
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

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
                    // Translate gamepad inputs into velocit
                    Vector2d input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad1.right_stick_x
                            )
                    );

                    //if the a button is pressed, then we convert to auto
                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

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


