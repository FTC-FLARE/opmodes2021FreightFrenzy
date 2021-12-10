package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Ducker {

    // this gives us access to all opMode information
    private LinearOpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    static final double MOTOR_POWER = 1.00;
    private DcMotor DuckerMotor= null;
    private final double TIMEOUT_TIME = 5;
    // Constructor
    public MM_Ducker(LinearOpMode opMode){
        this.opMode = opMode;
        DuckerMotor = opMode.hardwareMap.get(DcMotor.class, "Ducker");
        DuckerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void autoSpinRed(boolean redSide) {
        runtime.reset();
        if (redSide) {
            spinRed();
        } else {
            spinBlue();
        }

        while (opMode.opModeIsActive() && runtime.seconds() < TIMEOUT_TIME) {
            opMode.telemetry.addLine("Spinning Ducker");
        }
        stop();
    }

    public void manualSpin() {
        if (opMode.gamepad1.right_bumper) {
            spinBlue();
        } else if (opMode.gamepad1.left_bumper) {
            spinRed();
        } else {
            stop();
        }
    }

    private void stop() {
        DuckerMotor.setPower(0);
    }

    private void setPower(double motorPower) {
        DuckerMotor.setPower(motorPower);
    }

    private void spinRed() {
        setPower(-MOTOR_POWER);
    }

    private void spinBlue() {
        setPower(MOTOR_POWER);
    }
}