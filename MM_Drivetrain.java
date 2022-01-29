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
    private static final double STRAFE_P_COEFFICIENT = .03; //numerator is gain per degree error was .1
    private static final double STRAIGHT_P_COEFFICIENT = .095;
    private static final double RAMP_INTERVAL = 0.035;
    private static final double PIN_POWER_HIGH = 0.39;
    private static final double PIN_POWER_LOW = 0.35;
    private static final double CORRECTION_COEFFICIENT = 0.000775; //Gain per tick

    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveForwardInches(double forwardInches, double timeoutTime) {
        int leftTargetTicks = leftPriorEncoderTarget + inchesToTicks(forwardInches);
        int rightTargetTicks = rightPriorEncoderTarget + inchesToTicks(forwardInches);
        boolean lookingForTarget = true;
        robotHeading = priorAngleTarget;
//        rampUp = true;

        opMode.pLeftDriveController.setInputRange(leftPriorEncoderTarget, leftTargetTicks);
        opMode.pLeftDriveController.setSetpoint(leftTargetTicks);
        opMode.pRightDriveController.setInputRange(rightPriorEncoderTarget, rightTargetTicks);
        opMode.pRightDriveController.setSetpoint(rightTargetTicks);

        runtime.reset();
        while (opMode.opModeIsActive() && lookingForTarget && (runtime.seconds() < timeoutTime)) {
            setStraightPower();

            if (!opMode.pLeftDriveController.reachedTarget() && !opMode.pRightDriveController.reachedTarget()) {
                opMode.telemetry.addData("Left Distance", ticksToInches(opMode.pLeftDriveController.getCurrentError()));
                opMode.telemetry.addData("Right Distance", ticksToInches(opMode.pRightDriveController.getCurrentError()));
                opMode.telemetry.addData("Left Current", ticksToInches(leftCurrentTicks));
                opMode.telemetry.addData("Right Current", ticksToInches(rightCurrentTicks));
                opMode.telemetry.addData("Angle Heading", getCurrentHeading());
            } else {
                lookingForTarget = false;
            }
            opMode.telemetry.update();
        }
        stop();

        leftPriorEncoderTarget = leftTargetTicks;
        rightPriorEncoderTarget = rightTargetTicks;
    }

    public void strafeInches(double strafeInches, double timeoutTime) { //TODO Troubleshoot
        int backTargetTicks = inchesToTicks(strafeInches) + backPriorEncoderTarget;
        boolean lookingForTarget = true;
        rampUp = true;

        //same for all motors
        opMode.pBackDriveController.setInputRange(backPriorEncoderTarget, backTargetTicks);
        opMode.pBackDriveController.setSetpoint(backTargetTicks);

        runtime.reset();
        while (opMode.opModeIsActive() && lookingForTarget && (runtime.seconds() < timeoutTime)) {
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
            rampUp();
        }
        straighten(STRAIGHT_P_COEFFICIENT);
        setDrivePowers();
    }

    private void setStrafePower() {
        backCurrentTicks = -backEncoder.getCurrentPosition(); //TODO change to a port that reads the direction of the encoder count correctly

        double calculatedPower = opMode.pBackDriveController.calculatePower(backCurrentTicks);//removed min output

        flPower = calculatedPower;
        frPower = -calculatedPower;
        blPower = -calculatedPower;
        brPower = calculatedPower;

        encoderCorrect();//just for strafe for now

        if (rampUp) {
            rampUp();
        }
        straighten(STRAFE_P_COEFFICIENT);
        setDrivePowers();
    }

    private void straighten(double pCoefficient) {
        double headingError = calculateRotateError(priorAngleTarget);

        if (headingError != 0) {
            flPower = flPower - (headingError * pCoefficient * Math.abs(flPower));
            frPower = frPower + (headingError * pCoefficient * Math.abs(frPower));
            blPower = blPower - (headingError * pCoefficient * Math.abs(blPower));
            brPower = brPower + (headingError * pCoefficient * Math.abs(brPower));
        }
    }

    private void encoderCorrect() { //TODO RENAME
        //TODO maybe use Pcontroller?
        //biggest problem is calculate power adds the minimum
        leftCurrentTicks = leftEncoder.getCurrentPosition();
        rightCurrentTicks = rightEncoder.getCurrentPosition();

        double leftError = leftPriorEncoderTarget - leftCurrentTicks;
        double rightError =  rightPriorEncoderTarget - rightCurrentTicks;

        if (Math.abs(leftError) > 0 || Math.abs(rightError) > 0) { //modeled after straighten
            flPower = flPower + (leftError * CORRECTION_COEFFICIENT * Math.abs(flPower));
            frPower = frPower + (rightError * CORRECTION_COEFFICIENT * Math.abs(frPower));
            blPower = blPower + (leftError * CORRECTION_COEFFICIENT * Math.abs(blPower));
            brPower = brPower + (rightError * CORRECTION_COEFFICIENT * Math.abs(brPower));
        }
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
        setDrivePowers();
    }

    public void pRotateDegrees(double targetAngle){ //TODO add timeout proportional to distance (with drives)
        int leftStartingTicks = leftEncoder.getCurrentPosition();
        int rightStartingTicks = rightEncoder.getCurrentPosition();
        int backStartingTicks = backEncoder.getCurrentPosition();

        opMode.pTurnController.setInputRange(getCurrentHeading(), targetAngle);
        opMode.pTurnController.setSetpoint(targetAngle);
        do {
            double turnPower = Math.abs(opMode.pTurnController.calculatePower(getCurrentHeading()));

            if(translateAngle(opMode.pTurnController.getCurrentError()) > 0){
                startMotors(-turnPower, turnPower, -turnPower, turnPower);//rotate counter clockwise
            }else {
                startMotors(turnPower, -turnPower, turnPower, -turnPower);//rotate clockwise
            }

            opMode.telemetry.update();
        } while (opMode.opModeIsActive() && !opMode.pTurnController.reachedTarget());

        stop();
        leftPriorEncoderTarget = leftPriorEncoderTarget - leftStartingTicks + leftEncoder.getCurrentPosition();
        rightPriorEncoderTarget = rightPriorEncoderTarget - rightStartingTicks + rightEncoder.getCurrentPosition();
        backPriorEncoderTarget = backPriorEncoderTarget - backStartingTicks + backEncoder.getCurrentPosition();
        priorAngleTarget = targetAngle;
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
        driveForwardInches(forwardInches, 3);
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
        double strafeInches = 0;// temp drive until working strafe
        double rotateDegrees = 90;

        if((opMode.alliance == MM_OpMode.BLUE && opMode.startingPosition == MM_OpMode.WAREHOUSE) || (opMode.alliance == MM_OpMode.RED && opMode.startingPosition == MM_OpMode.STORAGE)) {

            strafeInches = 17;
            rotateDegrees = -90;

            //determine driving position
            if(duckPosition == 1) {
                forwardInches = -8.75;
            } else if(duckPosition == 2) {
                forwardInches = -6;
            }
        }else if((opMode.alliance == MM_OpMode.BLUE && opMode.startingPosition == MM_OpMode.STORAGE) || (opMode.alliance == MM_OpMode.RED && opMode.startingPosition == MM_OpMode.WAREHOUSE)) {

            forwardInches = -6;
            strafeInches = 22;

            //determine driving position (MEASURE)
            if (duckPosition == 1) {
                forwardInches = -6.75;
            } else if (duckPosition == 2) {
                forwardInches = -3.5;
            }
        }

        driveForwardInches(12, 5);
        pRotateDegrees(rotateDegrees);
        driveForwardInches(strafeInches,5);
        pRotateDegrees(179);
        driveForwardInches(forwardInches, 3);
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
            driveForwardInches(forwardInches, 7);
            pRotateDegrees(secondTargetHeading);
        }
    }

    public void warehousePark(){
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
        driveForwardInches(driveInches, 4);
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

    public void testMotors(){
        switchEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opMode.opModeIsActive()) {
            if (opMode.gamepad1.x) flPower = .5;
            else flPower = 0;

            if (opMode.gamepad1.y) frPower = .5;
            else frPower = 0;

            if (opMode.gamepad1.a) blPower = .5;
            else blPower = 0;

            if (opMode.gamepad1.b) brPower = .5;
            else brPower = 0;

            startMotors(flPower, frPower, blPower, brPower);
        }
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

    public float getCurrentHeading() {
        float heading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        opMode.telemetry.addData("Heading:", heading);
        return heading;
    }

    private double translateAngle(double angle) {
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    private double calculateRotateError(double targetHeading) {
        double headingError = targetHeading - getCurrentHeading();

        if (headingError > 180) {
            headingError -= 360;
        } else if (headingError < -180) {
            headingError += 360;
        }
        return headingError;
    }

    private void stop() {
    startMotors(0,0,0,0);
    }

    private void rampUp() { //TODO troubleshoot and optimize
        if (rampPower >= Math.abs(flPower)) {
            rampUp = false;
        }
        rampPower =  rampPower + RAMP_INTERVAL;
        if (flPower < 0) {
            flPower = -rampPower;
            frPower = rampPower;
            blPower = rampPower;
            brPower = -rampPower;
        } else {
            flPower = rampPower;
            frPower = -rampPower;
            blPower = -rampPower;
            brPower = rampPower;
        }
    }

    private void setDrivePowers() {
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
    private int inchesToTicks(double inches) {
        return (int) (inches * TICKS_PER_INCH);
    }

    private double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
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