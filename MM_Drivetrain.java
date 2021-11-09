package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MM_Drivetrain {
    private LinearOpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    static final double WHEEL_DIAMETER = 0;   // set this & use for calculating circumference
    static final double WHEEL_CIRCUMFERENCE = 0;   // use this to calculate ticks/inch
    static final double TICKS_PER_INCH = (1120 / 12.3684);   //odometry wheel ticks = 1440
    static final double DRIVE_SPEED = 0.5;

    private double ticks = 0;

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

        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        opMode.telemetry.addData("Status:", "Initialized Progbot");
        opMode.telemetry.update();
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
        setMotorPowers(flPower, frPower, blPower, brPower);
    }

    public int calculateTicks(double inches) {
        ticks = inches * TICKS_PER_INCH;
        return (int) ticks;
    }

    public void driveForwardInches(double Inches, double timeoutTime) {

        int frontLeftTargetInches;
        int frontRightTargetInches;
        int backLeftTargetInches;
        int backRightTargetInches;

        frontLeftTargetInches = frontLeftDrive.getCurrentPosition() + (int) (Inches * TICKS_PER_INCH);
        frontRightTargetInches = frontRightDrive.getCurrentPosition() + (int) (Inches * TICKS_PER_INCH);
        backLeftTargetInches = backLeftDrive.getCurrentPosition() + (int) (Inches * TICKS_PER_INCH);
        backRightTargetInches = backRightDrive.getCurrentPosition() + (int) (Inches * TICKS_PER_INCH);
        frontLeftDrive.setTargetPosition(frontLeftTargetInches);
        frontRightDrive.setTargetPosition(frontRightTargetInches);
        backLeftDrive.setTargetPosition(backLeftTargetInches);
        backRightDrive.setTargetPosition(backRightTargetInches);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        frontLeftDrive.setPower(Math.abs(DRIVE_SPEED));
        backLeftDrive.setPower(Math.abs(DRIVE_SPEED));
        frontRightDrive.setPower(Math.abs(DRIVE_SPEED));
        backRightDrive.setPower(Math.abs(DRIVE_SPEED));

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutTime) && (frontLeftDrive.isBusy() && frontRightDrive.isBusy()) && (backLeftDrive.isBusy() && backRightDrive.isBusy())) {

            // Display it for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d %7d %7d", frontLeftTargetInches, frontRightTargetInches, backLeftTargetInches, backRightTargetInches);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d %7d %7d",
                    frontLeftDrive.getCurrentPosition(),
                    frontRightDrive.getCurrentPosition(),
                    backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
            opMode.telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void strafeRightInches(double Inches, double timeoutTime) {

        int frontLeftTargetInches;
        int frontRightTargetInches;
        int backLeftTargetInches;
        int backRightTargetInches;

        frontLeftTargetInches = frontLeftDrive.getCurrentPosition() + (int) (Inches * TICKS_PER_INCH);
        frontRightTargetInches = frontRightDrive.getCurrentPosition() + (int) (-Inches * TICKS_PER_INCH);
        backLeftTargetInches = backLeftDrive.getCurrentPosition() + (int) (-Inches * TICKS_PER_INCH);
        backRightTargetInches = backRightDrive.getCurrentPosition() + (int) (Inches * TICKS_PER_INCH);
        frontLeftDrive.setTargetPosition(frontLeftTargetInches);
        frontRightDrive.setTargetPosition(frontRightTargetInches);
        backLeftDrive.setTargetPosition(backLeftTargetInches);
        backRightDrive.setTargetPosition(backRightTargetInches);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        frontLeftDrive.setPower(Math.abs(DRIVE_SPEED));
        backLeftDrive.setPower(Math.abs(DRIVE_SPEED));
        frontRightDrive.setPower(Math.abs(DRIVE_SPEED));
        backRightDrive.setPower(Math.abs(DRIVE_SPEED));

        while (opMode.opModeIsActive() && (runtime.seconds() < timeoutTime) && (frontLeftDrive.isBusy() && frontRightDrive.isBusy()) && (backLeftDrive.isBusy() && backRightDrive.isBusy())) {

            // Display it for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d %7d %7d", frontLeftTargetInches, frontRightTargetInches, backLeftTargetInches, backRightTargetInches);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d %7d %7d",
                    frontLeftDrive.getCurrentPosition(),
                    frontRightDrive.getCurrentPosition(),
                    backLeftDrive.getCurrentPosition(),
                    backRightDrive.getCurrentPosition());
            opMode.telemetry.update();
        }

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}

