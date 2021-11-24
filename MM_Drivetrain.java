package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MM_Drivetrain {
    private LinearOpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    private BNO055IMU gyro;
    private double ticks = 0;

    private boolean slowMode = false;
    private boolean slowModeIH = false;

    static final double WHEEL_DIAMETER = 0;   // set this & use for calculating circumference
    static final double WHEEL_CIRCUMFERENCE = 0;   // use this to calculate ticks/inch
    static final double TICKS_PER_INCH = (537.7 / 12.3684);   //odometry wheel ticks = 1440
    static final double DRIVE_SPEED = 0.2;
    static final double ANGLE_THRESHOLD = 1;

    public MM_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "FLMotor");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "FRMotor");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "BLMotor");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "BRMotor");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        gyro = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(gyroParameters);
    }

    public void setMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        frontLeftDrive.setPower(flPower);
        backLeftDrive.setPower(frPower);
        frontRightDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    public void driveWithSticks() {
        double drive = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;
        double strafe = opMode.gamepad1.left_stick_x;

        double flPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double frPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double blPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        double brPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

        if (opMode.gamepad1.a & !slowModeIH) {
            slowMode = !slowMode;
            slowModeIH = true;
        }

        else if (!opMode.gamepad1.a & slowModeIH) {
            slowModeIH = false;
        }

        if (slowMode) {
            flPower = flPower/2;
            frPower = frPower/2;
            blPower = blPower/2;
            brPower = brPower/2;
        }
        opMode.telemetry.addData("Slowmode", slowMode);
        opMode.telemetry.addData("SlowmodeIH", slowModeIH);
        setMotorPowers(flPower, frPower, blPower, brPower);
    }

    public int calculateTicks(double inches) {
        ticks = inches * TICKS_PER_INCH;
        return (int) ticks;
    }

    public void driveForwardInches(double Inches, double timeoutTime) {

        setTargetPosition(Inches);

        switchEncoderMode(false);

        runtime.reset();
        frontLeftDrive.setPower(Math.abs(DRIVE_SPEED));
        backLeftDrive.setPower(Math.abs(DRIVE_SPEED));
        frontRightDrive.setPower(Math.abs(DRIVE_SPEED));
        backRightDrive.setPower(Math.abs(DRIVE_SPEED));

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutTime) && (frontLeftDrive.isBusy() && frontRightDrive.isBusy()) && (backLeftDrive.isBusy() && backRightDrive.isBusy())) {

            // Display it for the driver.
            opMode.telemetry.addData("Path2", "Running at %7d :%7d %7d %7d",
                    frontLeftDrive.getCurrentPosition(),
                    frontRightDrive.getCurrentPosition(),
                    backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
            opMode.telemetry.update();
        }

        stop();

        switchEncoderMode(true);

    }
    public void rotate(double targetHeading, double timeoutTime) {
        double robotHeading;
        double headingError;
        boolean lookingForTarget = true;

        while (lookingForTarget) {
            //get heading value from gyro
            robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            headingError = targetHeading - robotHeading;

            //find error
            if (headingError > 180) {
                headingError -= 360;
            }

            else if (headingError < -180) {
                headingError += 360;
            }

            //determine whether the angle error is within threshold set
            if (headingError > ANGLE_THRESHOLD){
                frontLeftDrive.setPower(-DRIVE_SPEED);
                backLeftDrive.setPower(-DRIVE_SPEED);
                frontRightDrive.setPower(DRIVE_SPEED);
                backRightDrive.setPower(DRIVE_SPEED);
                opMode.telemetry.addData("Target Heading ", targetHeading);
                opMode.telemetry.addData("Robot Heading Error", headingError);
                opMode.telemetry.addData("Actual Robot Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                opMode.telemetry.update();
            }

            else if (headingError < -ANGLE_THRESHOLD){
                frontLeftDrive.setPower(DRIVE_SPEED);
                backLeftDrive.setPower(DRIVE_SPEED);
                frontRightDrive.setPower(-DRIVE_SPEED);
                backRightDrive.setPower(-DRIVE_SPEED);
                opMode.telemetry.addData("Target Heading ", targetHeading);
                opMode.telemetry.addData("Robot Heading Error", headingError);
                opMode.telemetry.addData("Actual Robot Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                opMode.telemetry.update();
            }

            else {
                lookingForTarget = false;
                stop();
            }
        }
    }
    public void strafeRightInches(double Inches, double timeoutTime) {
        setTargetPosition(Inches);

        switchEncoderMode(false);

        runtime.reset();
        frontLeftDrive.setPower(Math.abs(DRIVE_SPEED));
        backLeftDrive.setPower(Math.abs(DRIVE_SPEED));
        frontRightDrive.setPower(Math.abs(DRIVE_SPEED));
        backRightDrive.setPower(Math.abs(DRIVE_SPEED));

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutTime) && (frontLeftDrive.isBusy() && frontRightDrive.isBusy()) && (backLeftDrive.isBusy() && backRightDrive.isBusy())) {

            // Display it for the driver.
            opMode.telemetry.addData("Drive Inches", "Running to %7f ", Inches);
            opMode.telemetry.addData("Current Encoders", "Running at %7d :%7d %7d %7d",
                    frontLeftDrive.getCurrentPosition(),
                    frontRightDrive.getCurrentPosition(),
                    backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
            opMode.telemetry.update();
        }

        stop();

        switchEncoderMode(true);
    }

    public void diagonalDriveInches (double forwardInches, double leftInches, double timeoutTime) {

        double hypDistance;
        double targetHeading;
        double robotHeading;
        double headingError;
        boolean lookingForTarget = true;

        robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

        hypDistance = (Math.hypot(forwardInches, leftInches));

        targetHeading = Math.toDegrees(Math.atan2(leftInches, forwardInches));


        while (lookingForTarget) {

            //get heading value from gyro
            robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            headingError = targetHeading - robotHeading;

            //find error
            if (headingError > 180) {
                headingError -= 360;
            }

            else if (headingError < -180) {
                headingError += 360;
            }

            //determine whether the angle error is within threshold set
            if (headingError > ANGLE_THRESHOLD){
                frontLeftDrive.setPower(-DRIVE_SPEED);
                backLeftDrive.setPower(-DRIVE_SPEED);
                frontRightDrive.setPower(DRIVE_SPEED);
                backRightDrive.setPower(DRIVE_SPEED);
                opMode.telemetry.addData("Target Heading ", targetHeading);
                opMode.telemetry.addData("Robot Heading Error", headingError);
                opMode.telemetry.addData("Actual Robot Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                opMode.telemetry.addData("hypotenuse", hypDistance);
                opMode.telemetry.update();
            }

            else if (headingError < -ANGLE_THRESHOLD){
                frontLeftDrive.setPower(DRIVE_SPEED);
                backLeftDrive.setPower(DRIVE_SPEED);
                frontRightDrive.setPower(-DRIVE_SPEED);
                backRightDrive.setPower(-DRIVE_SPEED);
                opMode.telemetry.addData("Target Heading ", targetHeading);
                opMode.telemetry.addData("Robot Heading Error", headingError);
                opMode.telemetry.addData("Actual Robot Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
                opMode.telemetry.addData("hypotenuse", hypDistance);
                opMode.telemetry.update();
            }

            else {
                lookingForTarget = false;
                stop();
            }

        }

        driveForwardInches(hypDistance, timeoutTime);

    }

    public void deliveryDrive(double duckPosition) {

        double forwardInches = 0;

        if (duckPosition == 1) {
            forwardInches = 15.5;
        }

        if (duckPosition == 2 || duckPosition == 3) {
            forwardInches = 15.25;
        }

        driveForwardInches(forwardInches, 5);
        //diagonalDriveInches(0, -23, 7);
    }


    public void stop() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void setTargetPosition(double driveDistance) {
        int frontLeftTargetInches;
        int frontRightTargetInches;
        int backLeftTargetInches;
        int backRightTargetInches;

        frontLeftTargetInches = frontLeftDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);
        frontRightTargetInches = frontRightDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);
        backLeftTargetInches = backLeftDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);
        backRightTargetInches = backRightDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);

        frontLeftDrive.setTargetPosition(frontLeftTargetInches);
        frontRightDrive.setTargetPosition(frontRightTargetInches);
        backLeftDrive.setTargetPosition(backLeftTargetInches);
        backRightDrive.setTargetPosition(backRightTargetInches);
    }

    public void switchEncoderMode(boolean runToPosition) {

        if (runToPosition) {

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        else {

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

}

