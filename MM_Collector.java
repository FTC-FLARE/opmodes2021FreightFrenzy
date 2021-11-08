package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MM_Collector {

    private LinearOpMode opMode;


    private AnalogInput potentiometer = null;
    private DcMotor collector = null;
    private final double MAX_VOLTAGE = 3.3;

    public MM_Collector(LinearOpMode opMode) {
        this.opMode = opMode;

        collector = opMode.hardwareMap.get(DcMotor.class, "collector");
        potentiometer = opMode.hardwareMap.get(AnalogInput.class, "potentiometer");

    }

    public void runCollector() {

        double potVoltage = potentiometer.getVoltage();
        double motorPower = potVoltage / MAX_VOLTAGE;
        motorPower = motorPower / 2; // temp


        if (opMode.gamepad2.left_trigger > .1) {
            collector.setPower(1);
        } else if (opMode.gamepad2.right_trigger > .1) {
            collector.setPower(-1);
        } else {
            collector.setPower(0);
        }

        opMode.telemetry.addData("Voltage:", "%.2f", potVoltage);
        opMode.telemetry.addData("Motor power", "%.2f", motorPower);
        opMode.telemetry.update();
    }
}