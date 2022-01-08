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
    private double minPower = 0;
    private boolean rampUp = false;
    private double rampPrecentage = 0;
    private double leftDrivePower = 0;
    private double rightDrivePower = 0;
    private double leftDistanceError = 0;
    private double rightDistanceError = 0;
    private double backDistanceError = 0;
    private double pCoefficient = 0;

    private double leftEncoderTicks = 0;
    private double rightEncoderTicks = 0;
    private double backEncoderTicks = 0;
    private int leftPriorEncoderTarget = 0;
    private int rightPriorEncoderTarget = 0;
    private int backPriorEncoderTarget = 0;
    private int leftTargetTicks = 0;
    private int rightTargetTicks = 0;
    private int backTargetTicks = 0;

    // static final double WHEEL_CIRCUMFERENCE = 12.3684;  //4 inch wheels
    // static final double TICKS_PER_REVOLUTION = 537.7; //19.2 to 1 go builda

    private static final double WHEEL_DIAMETER = 1.496;  // odometry wheels
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    static final double TICKS_PER_REVOLUTION = 1440;
    static final double TICKS_PER_INCH = (TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE);  // 306.5499506
    static final double DRIVE_SPEED = 0.8;
    static final double ANGLE_THRESHOLD = 0.25;
    static final double DRIVE_THRESHOLD = 0.25 * TICKS_PER_INCH; //numerical value is # of inches
    static final double SLOW_DOWN_POINT = 24 * TICKS_PER_INCH; //numerical value is inches
    static final double ANGLE_P_COEFFICIENT = 1/100; //numerator is gain per degree error
    static final double RAMP_INTERVAL = 0.1;

    static final int RED = 1;
    static final int BLUE = 2;
    static final int WAREHOUSE = 1;
    static final int STORAGE = 2;

    public MM_Drivetrain(MM_OpMode opMode) {
        this.opMode = opMode;
        init();
    }

    public void driveForwardToPosition(double forwardInches, double timeoutTime) {

        if (forwardInches <= 24) {
            pCoefficient = 1/(SLOW_DOWN_POINT * (forwardInches/16));
            minPower = 0.12;
        } else if (forwardInches <=48){
            pCoefficient= 1/(SLOW_DOWN_POINT * (forwardInches/24));
            minPower = 0.09;
        } else {
            pCoefficient= 1/(SLOW_DOWN_POINT * (forwardInches/30));
            minPower = 0.08;
        }

        leftTargetTicks = inchesToTicks(forwardInches) + leftPriorEncoderTarget;
        rightTargetTicks = inchesToTicks(forwardInches) + rightPriorEncoderTarget;
        lookingForTarget = true;
        robotHeading = getCurrentHeading();

        runtime.reset();
        while (lookingForTarget && opMode.opModeIsActive() && (runtime.seconds() < timeoutTime)) {
            calculateDrivePowerAuto(true);
            assignMotorPowers(leftDrivePower, rightDrivePower, leftDrivePower, rightDrivePower);
            setDrivePowers();

            if (Math.abs(leftDistanceError) > DRIVE_THRESHOLD || Math.abs(rightDistanceError) > DRIVE_THRESHOLD) {
                opMode.telemetry.addData("Left Distance", ticksToInches(leftDistanceError));
                opMode.telemetry.addData("Right Distance", ticksToInches(rightDistanceError));
                opMode.telemetry.addData("Left Current", ticksToInches(leftEncoderTicks));
                opMode.telemetry.addData("Right Current", ticksToInches(rightEncoderTicks));
            } else {
                lookingForTarget = false;
            }
            opMode.telemetry.update();
        }
        stop();

        leftPriorEncoderTarget = leftTargetTicks;
        rightPriorEncoderTarget = rightTargetTicks;
    }

    public void strafeToPosition(double strafeInches, double timeoutTime) {
        backTargetTicks = inchesToTicks(strafeInches) + backPriorEncoderTarget;
        lookingForTarget = true;
        robotHeading = getCurrentHeading();

        runtime.reset();
        while (lookingForTarget && opMode.opModeIsActive() && (runtime.seconds() < timeoutTime)) {
            calculateDrivePowerAuto(false);
            assignMotorPowers(flPower, frPower, blPower, brPower);
            setDrivePowers();

            if (Math.abs(backDistanceError) > DRIVE_THRESHOLD) {
                opMode.telemetry.addData("Distance", ticksToInches(backDistanceError));
                opMode.telemetry.addData("Current", ticksToInches(frontLeftDrive.getCurrentPosition()));
            } else {
                stop();
                lookingForTarget = false;
            }
            opMode.telemetry.update();
        }
        backPriorEncoderTarget = backTargetTicks;
    }

    private void calculateDrivePowerAuto(boolean straight) {

        if (straight) {
            leftEncoderTicks = (leftEncoder.getCurrentPosition());
            rightEncoderTicks = (rightEncoder.getCurrentPosition());

//            rightEncoderTicks = leftEncoderTicks; //temporary
//
            leftDistanceError = leftTargetTicks - leftEncoderTicks;
            rightDistanceError = rightTargetTicks - rightEncoderTicks;

            leftDrivePower = pCoefficient * leftDistanceError;
            rightDrivePower = pCoefficient * rightDistanceError;

            //assign minimum drive power of 0.14 according to the higher power
            if (Math.abs(leftDrivePower) < minPower || Math.abs(rightDrivePower) < minPower) {
                double minimum = Math.min(Math.abs(leftDrivePower), Math.abs(rightDrivePower));
                leftDrivePower = ((1/minimum) * minPower * leftDrivePower);
                rightDrivePower = ((1/minimum) * minPower * rightDrivePower);
            }
            straighten(robotHeading);

            if (rampUp) {
                rampUp();
            }

        } else {
            backEncoderTicks = backEncoder.getCurrentPosition(); //encoder port 2
            backDistanceError = backTargetTicks - backEncoderTicks;

            flPower = pCoefficient * backDistanceError;
            frPower = -pCoefficient * backDistanceError;
            blPower = -pCoefficient * backDistanceError;
            brPower = pCoefficient * backDistanceError;
            straightenStrafe(robotHeading);
        }
    }

    private void straighten(double startHeading) {
        calculateRotateError(startHeading);

/*        if (headingError != 0) {
            rightDrivePower = rightDrivePower - (headingError * ANGLE_P_COEFFICENT * rightDrivePower);
            leftDrivePower = leftDrivePower + (headingError * ANGLE_P_COEFFICENT * leftDrivePower);
        }*/
    }

    private void straightenStrafe(double startHeading) {
        calculateRotateError(startHeading);

        if (headingError != 0) {
            flPower = flPower + (headingError * ANGLE_P_COEFFICIENT * flPower);
            frPower = frPower - (headingError * ANGLE_P_COEFFICIENT * frPower);
            blPower = blPower + (headingError * ANGLE_P_COEFFICIENT * blPower);
            brPower = brPower - (headingError * ANGLE_P_COEFFICIENT * brPower);

        }

    }

    public void driveWithSticks() {
        double drive = -opMode.gamepad1.left_stick_y;
        double turn = opMode.gamepad1.right_stick_x;
        double strafe = opMode.gamepad1.left_stick_x;

        flPower = drive + turn + strafe;
        frPower = drive - turn - strafe;
        blPower = drive + turn - strafe;
        brPower = drive - turn + strafe;

        setDrivePowers();
    }

    public void driveForwardInchesOld(double Inches, double timeoutTime) {
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

    public void strafeRightInchesOld(double Inches, double timeoutTime) {
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
    public void pRotateDegrees(double degrees){//timeout
        switchEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double targetAngle = translateAngle(getCurrentHeading() + degrees);

        opMode.pTurnController.setInputRange(0, Math.abs(degrees));
        opMode.pTurnController.setOutputRange(.12, .7);
        opMode.pTurnController.setSetpoint(targetAngle);
        do {
            double turnPower = opMode.pTurnController.getMinOutput() + opMode.pTurnController.calculatePower(getCurrentHeading());
            double translatedError = translateAngle(opMode.pTurnController.getCurrentError());

            if(translatedError > 0){
                startMotors(-turnPower, turnPower, -turnPower, turnPower);

            }else {
                startMotors(turnPower, -turnPower, turnPower, -turnPower);
            }

            opMode.telemetry.addData("Power to motors", turnPower);
            opMode.telemetry.update();
        }while (opMode.opModeIsActive() && !opMode.pTurnController.reachedTarget());
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

    public void diagonalDriveInchesOld(double forwardInches, double leftInches, double timeoutTime) {
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

        driveForwardInchesOld(hypDistance, timeoutTime);
    }

    public void driveToHub(int alliance, int startingPosition, double duckPosition) {
        //needs clean up with other autodrivemethods
        double forwardInches = 0;
        double strafeInches = 0;

        if((alliance == BLUE && startingPosition == WAREHOUSE) || (alliance == RED && startingPosition == STORAGE)) {

            forwardInches = -6;
            strafeInches = 30.5;

            //determine driving position
            if(duckPosition == 1) {
                forwardInches = -5.0;
            } else if(duckPosition == 2) {
                forwardInches = -2.5;
            }
        }else if((alliance == BLUE && startingPosition == STORAGE) || (alliance == RED && startingPosition == WAREHOUSE)) {

            forwardInches = -3;
            strafeInches = -28.5;

            //determine driving position (MEASURE)
            if (duckPosition == 1) {
                forwardInches = -7.25;
            } else if (duckPosition == 2) {
                forwardInches = -3.5;
            }
        }

        driveForwardToPosition(12, 5);
//        strafeRightInchesOld(strafeInches, 5);
        pRotateDegrees(179);
//        driveForwardToPosition(forwardInches, 3);
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
        driveForwardToPosition(forwardInches, 7);
        rotateToAngle(secondTargetHeading, 3);
    }

    public void outOfTheWay(int alliance){
        double angle = 90;
        double driveInches = -24;
        double strafeInches = -16;

        if (alliance == BLUE) {
            angle = -angle;
            strafeInches = -strafeInches;
        }

        rotateToAngle(angle, 5);
        driveForwardToPosition(driveInches, 3);
//        strafeRightInchesOld(strafeInches, 4);
    }
    public void odometryTelemetry() {
        opMode.telemetry.addData("Back Current", ticksToInches(frontLeftDrive.getCurrentPosition()));
        opMode.telemetry.addData("Right Current", ticksToInches(-frontRightDrive.getCurrentPosition()));
        opMode.telemetry.addData("Left Current", ticksToInches(-backLeftDrive.getCurrentPosition()));
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
        leftEncoder.setMode(runMode);
        rightEncoder.setMode(runMode);
        backEncoder.setMode(runMode);
    }

    private void rotateClockwise() {
        assignMotorPowers(0.2, -0.2, 0.2, -0.2);
        setDrivePowers();
    }

    private void rotateCounterClockwise() {
        assignMotorPowers(-0.2, 0.2, -0.2, 0.2);
        setDrivePowers();
    }

    public float getCurrentHeading() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double translateAngle(double angle) {
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
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

    private void rampUp() {
        rampPrecentage =  rampPrecentage + RAMP_INTERVAL;
        leftDrivePower = leftDrivePower * rampPrecentage;
        rightDrivePower = rightDrivePower * rampPrecentage;

        if (rampPrecentage == 1) {
            rampUp = false;
        }
    }


    private void setDriveSame(double motorPower) {
        assignMotorPowers(motorPower, motorPower, motorPower, motorPower);
        setDrivePowers();
    }

    private void assignMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        this.flPower = flPower;
        this.frPower = frPower;
        this.blPower = blPower;
        this.brPower = brPower;
    }

    private void setDrivePowers() {

        if (rampUp) {
            rampUp();
        } else {
            normalize();
            handleSlowMode();
        }

        startMotors(flPower, frPower, blPower, brPower);

        opMode.telemetry.addData("front left power:", flPower);
        opMode.telemetry.addData("front right power:", frPower);
        opMode.telemetry.addData("back left power:,", blPower);
        opMode.telemetry.addData("back right power:",  brPower);
    }

    private void startMotors(double flPower, double frPower, double blPower, double brPower) {
        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
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

    public void initializeGyro() {
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json";

        gyro = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(gyroParameters);
    }
}