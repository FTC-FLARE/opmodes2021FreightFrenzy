package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MM_Ducker {
    private MM_OpMode opMode;

    private DcMotor DuckerMotor= null;

    private final ElapsedTime runtime = new ElapsedTime();
    static final double MAX_POWER = 0.90;
    static final double MIN_POWER = 0.20;
    static final double RAMP_INTERVAL = 0.0025;
    private final double SPIN_TIME = 2;

    private double spinPower = MIN_POWER;

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
        DuckerMotor.setPower(-Range.clip(spinPower += RAMP_INTERVAL, MIN_POWER, MAX_POWER));
    }

    private void spinBlue() {
        DuckerMotor.setPower(Range.clip(spinPower += RAMP_INTERVAL, MIN_POWER, MAX_POWER));
    }

    private void stop() {
        DuckerMotor.setPower(0);
        spinPower = MIN_POWER;
    }
}
/*    public void manualSpinTEST(double minPower, double maxPower, double rampInterval) {
        if (opMode.gamepad1.right_bumper) {
            DuckerMotor.setPower(Range.clip(spinPower += rampInterval, minPower, maxPower));
        } else if (opMode.gamepad1.left_bumper) {
            DuckerMotor.setPower(-Range.clip(spinPower += rampInterval, minPower, maxPower));
        } else {
            stopTEST(minPower);
        }
    }

    private void stopTEST(double minPower) {
        DuckerMotor.setPower(0);
        spinPower = minPower;
    }*/