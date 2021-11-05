package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Slide {

    private LinearOpMode opMode;


    private AnalogInput potentiometer = null;
    private DigitalChannel bottomStop = null;
    private DistanceSensor topRange = null;

    private DcMotor arm = null;

    private final double MAX_VOLTAGE = 3.3;

    public MM_Slide(LinearOpMode opMode) {
        this.opMode = opMode;

        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        potentiometer = opMode.hardwareMap.get(AnalogInput.class, "potentiometer");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class,"bottomStop");
        topRange = opMode.hardwareMap.get(DistanceSensor.class,"topRange");

        bottomStop.setMode(DigitalChannel.Mode.INPUT);
    }

    public void runSlide() {

        double potVoltage = potentiometer.getVoltage();
        double motorPower = potVoltage / MAX_VOLTAGE;
        motorPower = motorPower; // temp

        if (potVoltage <= 0.5) {
            arm.setPower(0);
        } else if (opMode.gamepad1.left_bumper) {
            arm.setPower(motorPower);
        } else if (opMode.gamepad1.right_bumper) {
            arm.setPower(-motorPower);
        } else {
            arm.setPower(0);
        }

        opMode.telemetry.addData("bottom touch sensor",bottomStop.getState());
        opMode.telemetry.addData("top range sensor", topRange.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("Voltage:", "%.2f", potVoltage);
        opMode.telemetry.addData("Motor power", "%.2f", motorPower);
    }
}