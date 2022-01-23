package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MM_Ducker {
    private MM_OpMode opMode;

    private DcMotor DuckerMotor= null;

    private final ElapsedTime runtime = new ElapsedTime();
    static final double MAX_POWER = 0.75;
    static final double MIN_POWER = 0.40;
    static final double RAMP_INTERVAL = 0.01;
    private final double SPIN_TIME = 2.75;

    private double spinPower = 0;

    public MM_Ducker(MM_OpMode opMode){
        this.opMode = opMode;
        DuckerMotor = opMode.hardwareMap.get(DcMotor.class, "Ducker");
        DuckerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void autoSpin() {
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < SPIN_TIME) {
            if (opMode.alliance == MM_OpMode.BLUE){
                spinBlue();
            } else {
                spinRed();
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
        setPower(-Range.clip(spinPower += RAMP_INTERVAL, MIN_POWER, MAX_POWER));
    }

    private void spinBlue() {
        setPower(Range.clip(spinPower += RAMP_INTERVAL, MIN_POWER, MAX_POWER));
    }

    private void stop() {
        DuckerMotor.setPower(0);
        spinPower = 0.5;
    }

    private void setPower(double motorPower) {
        DuckerMotor.setPower(motorPower);
    }
}