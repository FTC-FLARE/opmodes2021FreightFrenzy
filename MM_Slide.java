package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Slide {

    enum transportPosition {
        COLLECT,
        lEVEL1,
        LEVEL2,
        LEVEL3,
        OTHER
    }

    private LinearOpMode opMode;
    private MM_Transporter transporter;

    private AnalogInput potentiometer = null;
    private DigitalChannel bottomStop = null;
    private DigitalChannel topStop = null;
    private DistanceSensor transportDown = null;
    private DistanceSensor transportUp = null;

    private DcMotor arm = null;
    private Servo shockAbsorber = null;

    private final double MAX_VOLTAGE = 3.3;
    private final int SCORE_LEVEL_1_PART_1 = 1100;
    private final int SCORE_LEVEL_1_PART_2 = 900;
    private final int SCORE_LEVEL_2 = 1800;
    private final int SCORE_LEVEL_3 = 2600;
    private final int CARRY_POSITION = 900;
    private final int COLLECT = 0;

    private transportPosition selectedPosition = transportPosition.COLLECT;

    private int levelOne = 0;

    public MM_Slide(LinearOpMode opMode) {
        this.opMode = opMode;

        transporter = new MM_Transporter(opMode, this);

        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shockAbsorber = opMode.hardwareMap.get(Servo.class, "shockAbsorber");
        shockAbsorber.setPosition(1); // down

        potentiometer = opMode.hardwareMap.get(AnalogInput.class, "potentiometer");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class, "bottomStop");//bottom limit switch on the slide
        topStop = opMode.hardwareMap.get(DigitalChannel.class, "topStop");//top limit switch on the slide
        bottomStop.setMode(DigitalChannel.Mode.INPUT);

        transportDown = opMode.hardwareMap.get(DistanceSensor.class, "transportDown");
        transportUp = opMode.hardwareMap.get(DistanceSensor.class, "transportUp");
    }

    public void runSlide() {
        double potVoltage = potentiometer.getVoltage();
        double motorPower = potVoltage / MAX_VOLTAGE;

//        if (potVoltage <= 0.5) {
//            arm.setPower(0);
//        } else if (opMode.gamepad2.right_bumper && arm.getCurrentPosition() < 3170) { //slide going up
//            arm.setPower(motorPower);
//        } else if (opMode.gamepad2.left_bumper && bottomStop.getState()) { //slide going down
//            arm.setPower(-motorPower);
//        }else if (!bottomStop.getState()){
//            arm.setPower(0);
//            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        else {
//            arm.setPower(0);
//        }

        if (opMode.gamepad2.left_stick_button) {
            goHome();

        }else if(opMode.gamepad1.left_bumper){
            goToPosition(CARRY_POSITION);
        }
        else if (opMode.gamepad2.a) {
            levelOne = 1;
            goToPosition(SCORE_LEVEL_1_PART_1);

        } else if (opMode.gamepad2.b) {
            goToPosition(SCORE_LEVEL_2);

        } else if (opMode.gamepad2.y) {
            goToPosition(SCORE_LEVEL_3);

        } else if (isTriggered(bottomStop)) {
//            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else if (opMode.gamepad2.x) {
            goToPosition(COLLECT);

        }else if (levelOne == 1 && !arm.isBusy()){
            levelOne = 2;
            goToPosition(SCORE_LEVEL_1_PART_2);

        }else if(levelOne == 2 && !arm.isBusy()){
            levelOne = 3;
        }

        transporter.controlFlip();

        opMode.telemetry.addData("arm encoder", arm.getCurrentPosition());
        opMode.telemetry.addData("transport up", transportUp.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("transport down", transportDown.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("bottom touch sensor", bottomStop.getState());
        opMode.telemetry.addData("top magnet sensor", topStop.getState());
        opMode.telemetry.addData("Voltage:", "%.2f", potVoltage);
        opMode.telemetry.addData("Motor power", "%.2f", motorPower);
    }

    private void goToPosition(int position) {
        if(isTriggered(topStop)){
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(0);

        }else{
            if(levelOne == 3){
                levelOne = 0;
            }
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(.5);
        }

    }


    public void goHome() {
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // ****** This needs to become state logic *****


        while (opMode.opModeIsActive() && !isTriggered(bottomStop)) {
            // ******************** MUST hold dpad_down *************************************
            transporter.controlFlip();
        }
        arm.setPower(.5);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(COLLECT);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getSlidePosition() {
        return arm.getCurrentPosition();
    }

    public boolean isTriggered(DigitalChannel sensor) {
        return !(sensor.getState());
    }

    public int getLevelOne() {
        return levelOne;
    }
}