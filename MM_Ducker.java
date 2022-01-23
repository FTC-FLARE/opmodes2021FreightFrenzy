package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Ducker {

    // this gives us access to all opMode information
    private MM_OpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    static final double MAX_POWER = 0.75;
    static final double MIN_POWER = 0.5;
    static final double RAMP_INTERVAL = 0.01;
    private DcMotor DuckerMotor= null;
    private final double TIMEOUT_TIME = 2.75;

    private double spinPower = 0;
    // Constructor
    public MM_Ducker(MM_OpMode opMode){
        this.opMode = opMode;
        DuckerMotor = opMode.hardwareMap.get(DcMotor.class, "Ducker");
        DuckerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void autoSpin() {
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < TIMEOUT_TIME) {
            if (opMode.alliance == MM_OpMode.RED){
                spinRed();
            }else if (opMode.alliance == MM_OpMode.BLUE){
                spinBlue();
            }
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

    private void spinRed() {
        setPower(-rampUpPower());
    }

    private void spinBlue() {
        setPower(rampUpPower());
    }

    private double rampUpPower() {
        if (spinPower <= MIN_POWER) {
            spinPower = MIN_POWER;
        } else if (spinPower >= MAX_POWER) {
            spinPower = MAX_POWER;
        } else {
            spinPower = spinPower + RAMP_INTERVAL;
        }
        return spinPower;
    }

    private void stop() {
        DuckerMotor.setPower(0);
        spinPower = 0;
    }

    private void setPower(double motorPower) {
        DuckerMotor.setPower(motorPower);
    }
}