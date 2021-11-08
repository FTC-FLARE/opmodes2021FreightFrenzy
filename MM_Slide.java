package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Slide {

    private LinearOpMode opMode;


    private AnalogInput potentiometer = null;
    private DigitalChannel bottomStop = null;
    private DistanceSensor topRange = null;
    private DistanceSensor transportDown = null;
    private DistanceSensor transportUp = null;
    private Servo transport = null;

    private DcMotor arm = null;

    private final double MAX_VOLTAGE = 3.3;
    private final double TRANSPORT_FLIP = 1000;
    private final double SCORE_LEVEL_1 = 2021;
    private final double SCORE_LEVEL_2 = 2021;
    private final double SCORE_LEVEL_3 = 2021;

    public MM_Slide(LinearOpMode opMode) {
        this.opMode = opMode;

        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transport = opMode.hardwareMap.get(Servo.class,"transport");

        potentiometer = opMode.hardwareMap.get(AnalogInput.class, "potentiometer");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class,"bottomStop");//bottom limit switch on the slide
        topRange = opMode.hardwareMap.get(DistanceSensor.class,"topRange");//top limit switch on the slide
        bottomStop.setMode(DigitalChannel.Mode.INPUT);

        transportDown = opMode.hardwareMap.get(DistanceSensor.class,"transportDown");
        transportUp = opMode.hardwareMap.get(DistanceSensor.class,"transportUp");


    }

    public void runSlide() {

        double potVoltage = potentiometer.getVoltage();
        double motorPower = potVoltage / MAX_VOLTAGE;


        if (potVoltage <= 0.5) {
            arm.setPower(0);
        } else if (opMode.gamepad2.right_bumper && arm.getCurrentPosition() < 3170) { //slide going up
            arm.setPower(motorPower);
        } else if (opMode.gamepad2.left_bumper && bottomStop.getState()) { //slide going down
            arm.setPower(-motorPower);
        }else if (!bottomStop.getState()){
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            arm.setPower(0);
        }

        if(arm.getCurrentPosition() > TRANSPORT_FLIP){//going up
            transport.setPosition(0);
        }else if (arm.getCurrentPosition() < TRANSPORT_FLIP){//going down
            transport.setPosition(1);
        }
        opMode.telemetry.addData("arm encoder", arm.getCurrentPosition());
        opMode.telemetry.addData("transport up",transportUp.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("transport down",transportDown.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("bottom touch sensor", bottomStop.getState());
        opMode.telemetry.addData("top range sensor", topRange.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("Voltage:", "%.2f", potVoltage);
        opMode.telemetry.addData("Motor power", "%.2f", motorPower);
    }
}