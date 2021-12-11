package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class MM_Drivetrain {
    private LinearOpMode opMode;

    private BNO055IMU gyro;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    private ElapsedTime runtime = new ElapsedTime();

    private double ticks = 0;
    private boolean slowMode = false;
    private boolean slowModeIsHandled = false;
    private double headingError;
    private double robotHeading;
    private boolean lookingForTarget;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;

    //4 inch wheels
    static final double TICKS_PER_INCH = (537.7 / 12.3684);  // odometry wheel ticks = 1440, 12.3684 is pi r^2
    static final double DRIVE_SPEED = 0.3;
    static final double ANGLE_THRESHOLD = 0.25;

    public MM_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveWithSticks() {
        double drive = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;
        double strafe = opMode.gamepad1.left_stick_x;

        flPower = drive + turn + strafe;
        frPower = drive + turn - strafe;
        blPower = drive - turn - strafe;
        brPower = drive - turn + strafe;

        normalize();
        handleSlowMode();

        setDrivePowers(flPower, frPower, blPower, brPower);
    }

    public void driveForwardInches(double Inches, double timeoutTime) {
        setTargetPosition(Inches);

        switchEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDriveSame(DRIVE_SPEED);

        runtime.reset();
        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutTime) && (frontLeftDrive.isBusy() && frontRightDrive.isBusy()) && (backLeftDrive.isBusy() && backRightDrive.isBusy())) {
           // Display it for the driver.
            opMode.telemetry.addData("Current ticks", "Running at %7d :%7d %7d %7d",
                    frontLeftDrive.getCurrentPosition(),
                    frontRightDrive.getCurrentPosition(),
                    backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
            opMode.telemetry.update();
        }

        stop();
        switchEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRightInches(double Inches, double timeoutTime) {
        setTargetPositionStrafe(Inches);
        switchEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDriveSame(DRIVE_SPEED);

        runtime.reset();
        //maybe change && to ||???????????
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
        switchEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void rotateToAngle(double targetHeading, double timeoutTime) {
        lookingForTarget = true;

        while (opMode.opModeIsActive() && lookingForTarget) {
            robotHeading = getCurrentHeading();
            calculateRotateError(targetHeading);

            //determine whether the angle error is within threshold set
            if (headingError > ANGLE_THRESHOLD){
                rotateCounterClockwise();
            } else if (headingError < -ANGLE_THRESHOLD){
                rotateClockwise();
            } else {
                lookingForTarget = false;
                stop();
            }

            opMode.telemetry.addData("Target Heading ", targetHeading);
            opMode.telemetry.addData("Robot Heading Error", headingError);
            opMode.telemetry.addData("Actual Robot Heading", robotHeading);
            opMode.telemetry.update();
        }
    }

    public void diagonalDriveInches(double forwardInches, double leftInches, double timeoutTime) {
        double hypDistance = (Math.hypot(forwardInches, leftInches));
        double targetHeading = Math.toDegrees(Math.atan2(leftInches, forwardInches));
        lookingForTarget = true;

        while (opMode.opModeIsActive() && lookingForTarget) {
            robotHeading = getCurrentHeading();
            calculateRotateError(targetHeading);

            //determine whether the angle error is within threshold set
            if (headingError > ANGLE_THRESHOLD) {
                rotateCounterClockwise();
            } else if (headingError < -ANGLE_THRESHOLD){
                rotateClockwise();
            } else {
                lookingForTarget = false;
                stop();
            }
            opMode.telemetry.addData("Target Heading ", targetHeading);
            opMode.telemetry.addData("Robot Heading Error", headingError);
            opMode.telemetry.addData("Actual Robot Heading", robotHeading);
            opMode.telemetry.addData("hypotenuse", hypDistance);
            opMode.telemetry.update();
        }

        driveForwardInches(hypDistance, timeoutTime);
    }

    public void driveToHub(String startPosition, double duckPosition) {
        //needs clean up with other autodrivemethods
        double forwardInches = 0;
        double strafeInches = 0;

        if (startPosition == "Blue Warehouse") {

            forwardInches = -6;
            strafeInches = 30.5;

            //determine driving position
            if (duckPosition == 1) {
                forwardInches = -5.0;
            } else if (duckPosition == 2) {
                forwardInches = -2.5;
            }
        }

        if (startPosition == "Blue Storage") {

            forwardInches = -3;
            strafeInches = -28.5;

            //determine driving position (MEASURE)
            if (duckPosition == 1) {
                forwardInches = -7.25;
            } else if (duckPosition == 2) {
                forwardInches = -3.5;
            }
        }

        driveForwardInches(12, 5);
        strafeRightInches(strafeInches, 5);
        rotateToAngle(180, 7);
        driveForwardInches(forwardInches, 3);
    }

    public void storagePark(boolean blueSide, double duckPosition, boolean storageStart) {
        //clean up later with dead encoders
        double targetHeading;
        //this variable is to straighten robot out after parking
        double secondTargetHeading;
        double forwardInches;

        forwardInches = 50;

        if (blueSide) {
            if (storageStart) {
                forwardInches = 55;
            }
            targetHeading = -80;;
            if (duckPosition == 1) {
                targetHeading = -82.75;
            } else if (duckPosition == 3){
                targetHeading = -80;
            }
            secondTargetHeading = -90;
        } else {
            if (!storageStart) {
                forwardInches = 47;
            } else {
                forwardInches = 49;
            }

            targetHeading = 78;
            secondTargetHeading = 90;

            if (duckPosition == 1) {
                targetHeading = 77;
            } else if (duckPosition == 3) {
                targetHeading = 80;
            }
        }

        rotateToAngle(targetHeading, 5);
        driveForwardInches(forwardInches, 7);
        rotateToAngle(secondTargetHeading, 3);
    }

    public void outOfTheWay(boolean blueSide) {

        double strafeInches;

        if (blueSide) {
            rotateToAngle(-90, 5);
        strafeInches = 16;
        } else { rotateToAngle(90, 5);
        strafeInches = -16;
        }

        driveForwardInches(-24, 3);
        strafeRightInches(strafeInches, 4);
    }

    private void setTargetPositionStrafe(double driveDistance) {
        int frontLeftTargetInches = frontLeftDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);
        int frontRightTargetInches = frontRightDrive.getCurrentPosition() - (int) (driveDistance * TICKS_PER_INCH);
        int backLeftTargetInches = backLeftDrive.getCurrentPosition() - (int) (driveDistance * TICKS_PER_INCH);
        int backRightTargetInches = backRightDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);

        frontLeftDrive.setTargetPosition(frontLeftTargetInches);
        frontRightDrive.setTargetPosition(frontRightTargetInches);
        backLeftDrive.setTargetPosition(backLeftTargetInches);
        backRightDrive.setTargetPosition(backRightTargetInches);
    }

    private void setTargetPosition(double driveDistance) {
        int frontLeftTargetInches = frontLeftDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);
        int frontRightTargetInches = frontRightDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);
        int backLeftTargetInches = backLeftDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);
        int backRightTargetInches = backRightDrive.getCurrentPosition() + (int) (driveDistance * TICKS_PER_INCH);

        frontLeftDrive.setTargetPosition(frontLeftTargetInches);
        frontRightDrive.setTargetPosition(frontRightTargetInches);
        backLeftDrive.setTargetPosition(backLeftTargetInches);
        backRightDrive.setTargetPosition(backRightTargetInches);
    }

    private void switchEncoderMode(DcMotor.RunMode runMode) {
            frontLeftDrive.setMode(runMode);
            frontRightDrive.setMode(runMode);
            backLeftDrive.setMode(runMode);
            backRightDrive.setMode(runMode);
    }

    private void rotateClockwise() {
        setDrivePowers(0.2, -0.2, 0.2, -0.2);
    }

    private void rotateCounterClockwise() {
        setDrivePowers(-0.2, 0.2, -0.2, 0.2);
    }

    private float getCurrentHeading() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private void calculateRotateError(double targetHeading) {
        headingError = targetHeading - robotHeading;

        if (headingError > 180) {
            headingError -= 360;
        } else if (headingError < -180) {
            headingError += 360;
        }
    }

    private void stop() {
        setDriveSame(0);
    }

    private void setDriveSame(double motorPower) {
        setDrivePowers(motorPower, motorPower, motorPower, motorPower);
    }

    private void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        frontLeftDrive.setPower(flPower);
        backLeftDrive.setPower(frPower);
        frontRightDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    private void handleSlowMode() {
        if (opMode.gamepad1.a & !slowModeIsHandled) {
            slowMode = !slowMode;
            slowModeIsHandled = true;
        } else if (!opMode.gamepad1.a & slowModeIsHandled) {
            slowModeIsHandled = false;
        }

        if (slowMode) {
            flPower = flPower/2;
            frPower = frPower/2;
            blPower = blPower/2;
            brPower = brPower/2;
        }
        opMode.telemetry.addData("Slowmode", slowMode);
    }

    private void normalize() {
        double max = Math.max(Math.abs(flPower), Math.abs(frPower));
        max = Math.max(max, Math.abs(blPower));
        max =  Math.max(max, Math.abs(brPower));

        if (max > 1) {
            flPower = flPower/max;
            frPower = frPower/max;
            blPower = blPower/max;
            brPower = brPower/max;
        }
    }
    private int calculateTicks(double inches) {
        ticks = inches * TICKS_PER_INCH;
        return (int) ticks;
    }

    private void init() {
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

        switchEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switchEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initalizeGyro() {
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        gyro = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(gyroParameters);
    }
}