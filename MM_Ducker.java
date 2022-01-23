package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MM_Ducker {

    // this gives us access to all opMode information
    private MM_OpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    static final double MAX_POWER = 0.75;
    static final double MIN_POWER = 0.40;
    static final double RAMP_INTERVAL = 0.01;
    static final double RED_SIDE = -1;
    static final double BLUE_SIDE = 1;
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
                rampUpSpin(RED_SIDE);
            }else if (opMode.alliance == MM_OpMode.BLUE){
                rampUpSpin(BLUE_SIDE);
            }
            opMode.telemetry.addLine("Spinning Ducker");
        }
        stop();
    }

    public void manualSpin() {
        if (opMode.gamepad1.right_bumper) {
            rampUpSpin(BLUE_SIDE);
        } else if (opMode.gamepad1.left_bumper) {
            rampUpSpin(RED_SIDE);
        } else {
            stop();
        }
    }

    private void rampUpSpin(double allianceSideMultiplier) {
        spinPower = spinPower + RAMP_INTERVAL;
        Range.clip(spinPower, MIN_POWER, MAX_POWER);
        spinPower = spinPower * allianceSideMultiplier;
        setPower(spinPower);
    }

    private void stop() {
        DuckerMotor.setPower(0);
        spinPower = 0.5;
    }

    private void setPower(double motorPower) {
        DuckerMotor.setPower(motorPower);
    }
    /*    private void spinRed() {
        rampUpSpin(RED_SIDE);
    }

    private void spinBlue() {
        rampUpSpin(BLUE_SIDE);
    }*/
}