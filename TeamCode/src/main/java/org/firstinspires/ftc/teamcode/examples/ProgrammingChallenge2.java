package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


public class ProgrammingChallenge2 extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //name imu,
    BNO055IMU imu;
    Orientation angles;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    //name gear ratio wheel radius and ticks for encoder cm conversion
    public static double GEAR_RATIO = 1.0; // for simulator
    public static double WHEEL_RADIUS = 5.0;  // 5 cm
    public static double TICKS_PER_ROTATION = 1120.0;  // From NeveRest (for simulator)

    //make it much easier to name powers
    private void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        double largest = 1;
        largest = Math.max(largest, Math.abs(flSpeed));
        largest = Math.max(largest, Math.abs(frSpeed));
        largest = Math.max(largest, Math.abs(blSpeed));
        largest = Math.max(largest, Math.abs(brSpeed));

        frontLeft.setPower(flSpeed / largest);
        frontRight.setPower(frSpeed / largest);
        backLeft.setPower(blSpeed / largest);
        backRight.setPower(brSpeed / largest);
    }

    public void MecanumEncoder(double power, int fowardcm, int strafecm) {

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //find cm per tick based on 2pirad/tick per revolution
        double CM_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS) / TICKS_PER_ROTATION;
        //get tick we need to travel based on cm
        int tick1 = (int) (fowardcm / CM_PER_TICK);
        int tick2 = (int) (strafecm / CM_PER_TICK);

        //reset encoders
        frontLeft.setMode(STOP_AND_RESET_ENCODER);
        frontRight.setMode(STOP_AND_RESET_ENCODER);
        backLeft.setMode(STOP_AND_RESET_ENCODER);
        backRight.setMode(STOP_AND_RESET_ENCODER);

        int frontLeftTick = tick1 + tick2;
        int frontRightTick = tick1 - tick2;
        int backLeftTick = tick1 - tick2;
        int backRightTick = tick1 + tick2;

        frontLeft.setTargetPosition(frontLeftTick);
        frontRight.setTargetPosition(frontRightTick);
        backLeft.setTargetPosition(backLeftTick);
        backRight.setTargetPosition(backRightTick);

        //run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //take the power, reverse it for the top right and bottom left wheels to strafe left

        setSpeeds(power, power, power, power);

        while (frontRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
        }

        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        sleep(250);


    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void rotate(int degrees, double power) {
        degrees = -degrees;

        double leftPower, rightPower;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        resetAngle();

        if (degrees < 0) {
            //reverse these two because its backwards
            leftPower = power;
            rightPower = -power;

        } else if (degrees > 0) {
            leftPower = -power;
            rightPower = power;
        } else return;

        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);

        if (degrees < 0) {
            while (opModeIsActive() && getAngle() == 0) {
            }
            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else
            while (opModeIsActive() && getAngle() == 0) {
            }
        while (opModeIsActive() && getAngle() < degrees) {
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        sleep(750);
        resetAngle();
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }


    //override, declare the actual running part
    @Override
    public void runOpMode() {

        //add motors, run with encoder, add arm and servo
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hardwareMap.dcMotor.get("back_right_motor");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reverse direction of left motors
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //declare breaking
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //declare IMU information
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //wait for start
        waitForStart();

        //show that its running
        telemetry.addData("Mode", "running...");
        telemetry.update();

        MecanumEncoder(.5, 0, 50);
        MecanumEncoder(.5, 75, 0);
        MecanumEncoder(.5, -5, 0);
        MecanumEncoder(.5, 0, -200);
        MecanumEncoder(.5, 5, 0);
        MecanumEncoder(.5, -5, 0);
        MecanumEncoder(.5, 0, 200);
        MecanumEncoder(.5, 5, 0);
        //hello this is a test

    }

}
