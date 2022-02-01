package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class MM_Drivetrain {
    private MM_OpMode opMode;
    //TODO start new class for utilities
    private BNO055IMU gyro;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotorEx leftEncoder = null;
    private DcMotorEx rightEncoder = null;
    private DcMotorEx backEncoder = null;
    private Servo odometryLeft = null;
    private Servo odometryRight = null;
    private Servo odometryBack = null;

    private final ElapsedTime runtime = new ElapsedTime();

    private boolean slowMode = false;
    private boolean slowModeIsHandled = false;
    private double robotHeading;

    private double flPower = 0;
    private double frPower = 0;
    private double blPower = 0;
    private double brPower = 0;
    private boolean rampUp = false;
    private double rampPower = 0;
    private double leftDrivePower = 0;
    private double rightDrivePower = 0;

    private int leftCurrentTicks = 0;
    private int rightCurrentTicks = 0;
    private int backCurrentTicks = 0;
    private int leftPriorEncoderTarget = 0;
    private int rightPriorEncoderTarget = 0;
    private int backPriorEncoderTarget = 0;
    private double priorAngleTarget = 0;

    private static final double WHEEL_DIAMETER = 1.496;  // odometry wheels in inches
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private static final double TICKS_PER_REVOLUTION = 1440;
    private static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE);  // 306.5499506
    private static final double STRAFE_P_COEFFICIENT = .09; //numerator is gain per degree error was .1
    private static final double STRAIGHT_P_COEFFICIENT = .095;
    private static final double RAMP_INTERVAL = 0.035;
    private static final double PIN_POWER_HIGH = 0.39;
    private static final double PIN_POWER_LOW = 0.35;
    private static final double SECONDS_PER_INCH = 0.08;
    private static final double SECONDS_PER_DEGREE = 0.025;
    private static final double CORRECTION_COEFFICIENT = 0.000975; //Gain per tick

    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveForwardInches(double inches) {
        int leftTargetTicks = leftPriorEncoderTarget + inchesToTicks(inches);
        int rightTargetTicks = rightPriorEncoderTarget + inchesToTicks(inches);
        boolean lookingForTarget = true;
        robotHeading = priorAngleTarget;
//        rampUp = true;

        opMode.pLeftDriveController.setInputRange(leftPriorEncoderTarget, leftTargetTicks);
        opMode.pLeftDriveController.setSetpoint(leftTargetTicks);
        opMode.pRightDriveController.setInputRange(rightPriorEncoderTarget, rightTargetTicks);
        opMode.pRightDriveController.setSetpoint(rightTargetTicks);

        runtime.reset();
        while (opMode.opModeIsActive() && lookingForTarget && (runtime.seconds() < calculateTimeout(SECONDS_PER_INCH, inches, 2.5))) {
            setStraightPower();

            if (!opMode.pLeftDriveController.reachedTarget() && !opMode.pRightDriveController.reachedTarget()) {
                opMode.telemetry.addData("Left Distance", ticksToInches(opMode.pLeftDriveController.getCurrentError()));
                opMode.telemetry.addData("Right Distance", ticksToInches(opMode.pRightDriveController.getCurrentError()));
                opMode.telemetry.addData("Left Current", ticksToInches(leftCurrentTicks));
                opMode.telemetry.addData("Right Current", ticksToInches(rightCurrentTicks));
/*                opMode.telemetry.addData("Angle Heading", getCurrentHeading());
                opMode.telemetry.addData("Final runtime", runtime.seconds());*/
            } else {
                lookingForTarget = false;
            }
            opMode.telemetry.update();
        }
        stop();

        leftPriorEncoderTarget = leftTargetTicks;
        rightPriorEncoderTarget = rightTargetTicks;
    }

    public void strafeInches(double inches) { //TODO Troubleshoot
        int backTargetTicks = inchesToTicks(inches) + backPriorEncoderTarget;
        boolean lookingForTarget = true;

        //same for all motors
        opMode.pBackDriveController.setInputRange(backPriorEncoderTarget, backTargetTicks);
        opMode.pBackDriveController.setSetpoint(backTargetTicks);
        rampPower = 0;
        runtime.reset();
        while (opMode.opModeIsActive() && lookingForTarget && (runtime.seconds() < calculateTimeout(SECONDS_PER_INCH, inches, 2.5))) {
            setStrafePower();

            if (!opMode.pBackDriveController.reachedTarget()) {
                opMode.telemetry.addData("Distance", ticksToInches(opMode.pBackDriveController.getCurrentError()));
                opMode.telemetry.addData("Current", ticksToInches(backCurrentTicks));
                opMode.telemetry.addData("Angle Heading", getCurrentHeading());
            } else {
                stop();
                lookingForTarget = false;
            }
            opMode.telemetry.update();
        }
        backPriorEncoderTarget = backTargetTicks;
    }

    public void pRotateDegrees(double targetAngle){ //TODO test odd angles
        int leftStartingTicks = leftEncoder.getCurrentPosition();
        int rightStartingTicks = rightEncoder.getCurrentPosition();
        int backStartingTicks = backEncoder.getCurrentPosition();
        double timeOut = calculateTimeout(SECONDS_PER_DEGREE, targetAngle - getCurrentHeading(), 2);

        opMode.pTurnController.setInputRange(getCurrentHeading(), targetAngle);
        opMode.pTurnController.setSetpoint(targetAngle);
        runtime.reset();
        do {
            double turnPower = Math.abs(opMode.pTurnController.calculatePower(getCurrentHeading()));

            if(translateAngle(opMode.pTurnController.getCurrentError()) > 0){
                startMotors(-turnPower, turnPower, -turnPower, turnPower);//rotate counter clockwise
            }else {
                startMotors(turnPower, -turnPower, turnPower, -turnPower);//rotate clockwise
            }
            opMode.telemetry.addData("time out time", timeOut);
            opMode.telemetry.addData("reached target", opMode.pTurnController.reachedTarget());
            opMode.telemetry.update();
        } while (opMode.opModeIsActive() && !opMode.pTurnController.reachedTarget() && runtime.seconds() < timeOut);
        if (opMode.pTurnController.reachedTarget()) {
            stop();
            opMode.telemetry.update();
        }
        leftPriorEncoderTarget = leftPriorEncoderTarget - leftStartingTicks + leftEncoder.getCurrentPosition();
        rightPriorEncoderTarget = rightPriorEncoderTarget - rightStartingTicks + rightEncoder.getCurrentPosition();
        backPriorEncoderTarget = backPriorEncoderTarget - backStartingTicks + backEncoder.getCurrentPosition();
        priorAngleTarget = targetAngle;
    }

    private void setStraightPower() {
        leftCurrentTicks = (leftEncoder.getCurrentPosition());
        rightCurrentTicks = (rightEncoder.getCurrentPosition());

        leftDrivePower = opMode.pLeftDriveController.calculatePower(leftCurrentTicks);
        rightDrivePower = opMode.pRightDriveController.calculatePower(rightCurrentTicks);

        flPower = leftDrivePower;
        frPower = rightDrivePower;
        blPower = leftDrivePower;
        brPower = rightDrivePower;

        if (rampUp) {
            rampUpStrafe(leftDrivePower);
        }
        straighten(STRAIGHT_P_COEFFICIENT, leftDrivePower, rightDrivePower);
        normalize();
        startMotors(flPower, frPower, blPower, brPower);
    }

    private void setStrafePower() {
        backCurrentTicks = -backEncoder.getCurrentPosition(); //TODO change to a port that reads the direction of the encoder count correctly

        double calculatedPower = opMode.pBackDriveController.calculatePower(backCurrentTicks);//removed min output
        if (rampPower <= Math.abs(calculatedPower)) {
            rampPower =  rampPower + RAMP_INTERVAL;
            if (calculatedPower < 0) {
                calculatedPower = -rampPower;
            } else {
                calculatedPower = rampPower;
            }
        }

        flPower = calculatedPower;
        frPower = -calculatedPower;
        blPower = -calculatedPower;
        brPower = calculatedPower;

        encoderCorrect(calculatedPower);//just for strafe for now
        straighten(STRAFE_P_COEFFICIENT, calculatedPower, calculatedPower);
        normalize();
        startMotors(flPower, frPower, blPower, brPower);
    }

    private void rampUpStrafe(double currentPower) { //TODO troubleshoot and optimize


    }

    private void straighten(double pCoefficient, double leftCalculatedPower, double rightCalculatedPower) {
        double headingError = translateAngle(priorAngleTarget - getCurrentHeading());

        if (headingError != 0) {
            flPower = flPower - (headingError * pCoefficient * Math.abs(leftCalculatedPower));
            frPower = frPower + (headingError * pCoefficient * Math.abs(rightCalculatedPower));
            blPower = blPower - (headingError * pCoefficient * Math.abs(leftCalculatedPower));
            brPower = brPower + (headingError * pCoefficient * Math.abs(rightCalculatedPower));
        }
    }

    private void encoderCorrect(double calculatedPower) { //TODO RENAME
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();

        double leftError = leftPriorEncoderTarget - leftCurrentTicks;
        double rightError =  rightPriorEncoderTarget - rightCurrentTicks;

        if (Math.abs(leftError) > 0 || Math.abs(rightError) > 0) { //modeled after straighten
            flPower = flPower + (leftError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            frPower = frPower + (rightError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            blPower = blPower + (leftError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
            brPower = brPower + (rightError * CORRECTION_COEFFICIENT * Math.abs(calculatedPower));
        }
    }

    public void driveToCarousel(double duckPosition) {
        double forwardInches = 42;
        double targetAngle = 103;
        double secondTargetAngle = -37;
        double timeoutTime = 0.90; //default for red 3
        if (duckPosition == 1 && opMode.alliance == MM_OpMode.RED) {
            secondTargetAngle = -27;
        }
         if (opMode.alliance == MM_OpMode.BLUE) {
            forwardInches = 44;
            targetAngle = -100;
            secondTargetAngle = 20;
            timeoutTime = 0.75; //default for blue 3
        }

        pRotateDegrees(targetAngle);
        driveForwardInches(forwardInches);
        pRotateDegrees(secondTargetAngle);

        if (duckPosition == 2) {
            timeoutTime = 0.9;
        } else if (duckPosition == 1) {
            timeoutTime = 1.1;
        }

        runtime.reset();
        startMotors(-0.2, -0.2, -0.2,-0.2);
        while (opMode.opModeIsActive() && runtime.seconds() < timeoutTime) {
        }
        stop();
    }

    public void parkFromCarousel() {
        runtime.reset();
        startMotors(0.2, 0.2, 0.2,0.2);
        while (opMode.opModeIsActive() && runtime.seconds() < 1) {
        }
        stop();
    }

    public void driveToHub(double duckPosition) {
        //needs clean up with other autodrivemethods
        double forwardInches = -6;
        double strafeInches = 0;

        if((opMode.alliance == MM_OpMode.BLUE && opMode.startingPosition == MM_OpMode.WAREHOUSE) || (opMode.alliance == MM_OpMode.RED && opMode.startingPosition == MM_OpMode.STORAGE)) {

            strafeInches = 16;

            //determine driving position
            if(duckPosition == 1) {
                forwardInches = -8.75;
            } else if(duckPosition == 2) {
                forwardInches = -6;
            }
        }else if((opMode.alliance == MM_OpMode.BLUE && opMode.startingPosition == MM_OpMode.STORAGE) || (opMode.alliance == MM_OpMode.RED && opMode.startingPosition == MM_OpMode.WAREHOUSE)) {

            forwardInches = -6;
            strafeInches = -27;

            //determine driving position (MEASURE)
            if (duckPosition == 1) {
                forwardInches = -6.75;
            } else if (duckPosition == 2) {
                forwardInches = -3.5;
            }
        }

        driveForwardInches(12);
        strafeInches(strafeInches);
        pRotateDegrees(179);
        driveForwardInches(forwardInches);
    }

    public void storagePark(double duckPosition) {
        if (opMode.spinDucker) {
            parkFromCarousel();
        } else {
            //clean up later with dead encoders
            double targetHeading;
            //this variable is to straighten robot out after parking
            double secondTargetHeading;
            double forwardInches;

            forwardInches = 50;

            if (opMode.alliance == MM_OpMode.BLUE) {
                if (opMode.startingPosition == MM_OpMode.STORAGE) {
                    forwardInches = 47;
                }
                targetHeading = -80;
                if (duckPosition == 1) {
                    targetHeading = -82.75;
                } else if (duckPosition == 3) {
                    targetHeading = -80;
                }
                secondTargetHeading = -90;
            } else {
                if (opMode.startingPosition == MM_OpMode.WAREHOUSE) {
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

            pRotateDegrees(targetHeading);
            driveForwardInches(forwardInches);
            pRotateDegrees(secondTargetHeading);
        }
    }

    public void warehousePark() {
        double angle = -105;
        double driveInches = 18;
        double motorPowers = 0.3;
        double rightMotorPowers = 0.3;
        double leftMotorPowers = 0.32;

        if (opMode.alliance == MM_OpMode.BLUE) {
            angle = -angle;
            motorPowers = -motorPowers;
            rightMotorPowers = 0.32;
            leftMotorPowers = 0.3;
        }

        pRotateDegrees(angle);
        driveForwardInches(driveInches);
        initOdometryServos(1);

        runtime.reset();
        startMotors(motorPowers, -motorPowers, -motorPowers, motorPowers);
        while (opMode.opModeIsActive() && runtime.seconds() < 3) {
        }
        stop();

        runtime.reset();
        startMotors(leftMotorPowers, rightMotorPowers, leftMotorPowers, rightMotorPowers);
        while (opMode.opModeIsActive() && runtime.seconds() < 3) {
        }
        stop();
    }

    public void driveWithSticks() {
        if (opMode.gamepad1.left_trigger > 0) {
            startMotors(PIN_POWER_LOW, PIN_POWER_HIGH, PIN_POWER_LOW, PIN_POWER_HIGH);
        } else if (opMode.gamepad1.right_trigger > 0) {
            startMotors(PIN_POWER_HIGH, PIN_POWER_LOW, PIN_POWER_HIGH, PIN_POWER_LOW);
        } else {
            double drive = -opMode.gamepad1.left_stick_y;
            double turn = opMode.gamepad1.right_stick_x;
            double strafe = opMode.gamepad1.left_stick_x;

            flPower = drive + turn + strafe;
            frPower = drive - turn - strafe;
            blPower = drive + turn - strafe;
            brPower = drive - turn + strafe;
        }
        normalize();
        handleSlowMode();
        startMotors(flPower, frPower, blPower, brPower);
    }

    private void startMotors(double flPower, double frPower, double blPower, double brPower) {
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
        frontLeftDrive.setPower(flPower);

        opMode.telemetry.addData("front left power:", flPower);
        opMode.telemetry.addData("front right power:", frPower);
        opMode.telemetry.addData("back left power:,", blPower);
        opMode.telemetry.addData("back right power:",  brPower);
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

    private double translateAngle(double angle) {
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    public float getCurrentHeading() {
        float heading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        opMode.telemetry.addData("Heading:", heading);
        return heading;
    }

    private double calculateTimeout(double pValue, double distance, double min) {
        opMode.telemetry.addData("Calculated Time", Math.max(min, Math.abs(pValue * distance)));
        return Math.max(min, Math.abs(pValue * distance));
    }

    private void stop() {
        startMotors(0,0,0,0);
    }

    private int inchesToTicks(double inches) {
        return (int) (inches * TICKS_PER_INCH);
    }

    private double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    public void odometryTelemetry() {
        opMode.telemetry.addData("Back Current", ticksToInches(backEncoder.getCurrentPosition()));
        opMode.telemetry.addData("Right Current", ticksToInches(rightEncoder.getCurrentPosition()));
        opMode.telemetry.addData("Left Current", ticksToInches(leftEncoder.getCurrentPosition()));
    }

    private void switchEncoderMode(DcMotor.RunMode runMode) {
        frontLeftDrive.setMode(runMode);
        frontRightDrive.setMode(runMode);
        backLeftDrive.setMode(runMode);
        backRightDrive.setMode(runMode);
        leftEncoder.setMode(runMode);
        rightEncoder.setMode(runMode);
        backEncoder.setMode(runMode);
    }

    private void init() {
        frontLeftDrive = opMode.hardwareMap.get(DcMotor.class, "FLMotor");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "FRMotor");
        backLeftDrive = opMode.hardwareMap.get(DcMotor.class, "BLMotor");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "BRMotor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftEncoder = opMode.hardwareMap.get(DcMotorEx.class, "FRMotor");  // was BL
        rightEncoder = opMode.hardwareMap.get(DcMotorEx.class, "BLMotor");  // was FR
        backEncoder = opMode.hardwareMap.get(DcMotorEx.class, "FLMotor");

        switchEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        switchEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initOdometryServos(double position) {
        odometryLeft = opMode.hardwareMap.get(Servo.class, "OdomLeft");
        odometryRight = opMode.hardwareMap.get(Servo.class, "OdomRight");
        odometryBack = opMode.hardwareMap.get(Servo.class, "OdomBack");
        odometryLeft.setPosition(position);
        odometryBack.setPosition(position);
        if (position == 1) {
            odometryRight.setPosition(0);
        }else {
            odometryRight.setPosition(1);
        }
    }

    public void initializeGyro() {
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        gyro = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(gyroParameters);
    }
}