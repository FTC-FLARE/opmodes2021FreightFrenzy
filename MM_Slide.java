package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.comp.Todo;

import java.util.function.ToDoubleBiFunction;

public class MM_Slide {

    enum TransportPosition {
        COLLECT(0),
        LEVEL1_PART_1(1700),
        LEVEL1_PART_2(1230),
        LEVEL2(2780),
        LEVEL3(4025),
        MAX(4670);

        public final int ticks;

        private TransportPosition(int ticks) {
            this.ticks = ticks;
        }
    }

    private LinearOpMode opMode;
    private MM_Transporter transporter = null;

    private AnalogInput potentiometer = null;
    private DigitalChannel bottomStop = null;
    private DigitalChannel topStop = null;

    private DcMotor arm = null;
    private Servo shockAbsorber = null;

    private double UP_POWER = 1;
    private double DOWN_POWER = 0.65;
//    private int SLEEP_TIME = 1500;

    private TransportPosition selectedPosition = TransportPosition.COLLECT;

    private int levelOne = 0;
    private boolean manualSlide = false;
    private boolean isHandled = false;
    private boolean headedUp = false;
    private boolean shockAbsorberEngaged = true;

    public MM_Slide(LinearOpMode opMode) {
        this.opMode = opMode;

        transporter = new MM_Transporter(opMode, this);

        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shockAbsorber = opMode.hardwareMap.get(Servo.class, "shockAbsorber");
        shockAbsorber.setPosition(0); // engage

        potentiometer = opMode.hardwareMap.get(AnalogInput.class, "potentiometer");
        bottomStop = opMode.hardwareMap.get(DigitalChannel.class, "bottomStop");//bottom limit switch on the slide
        topStop = opMode.hardwareMap.get(DigitalChannel.class, "topStop");//top limit switch on the slide
        bottomStop.setMode(DigitalChannel.Mode.INPUT);
    }

    public void runSlide() {
        if (opMode.gamepad2.right_trigger > .1) { // slide request up
            if (shockAbsorberEngaged) { // lift shock absorber before starting up
                shockAbsorber.setPosition(1);
                shockAbsorberEngaged = false;
            }
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (arm.getCurrentPosition() < TransportPosition.MAX.ticks) {
                if (!isTriggered(topStop)){
                    arm.setPower(opMode.gamepad2.right_trigger);
                    // add magnet logic
                }else {
                    arm.setPower(0);
                }
            }
            headedUp = true;
            manualSlide = true;
            isHandled = false;

        } else if (opMode.gamepad2.left_trigger > .1) { // slide request down
            if (isTriggered(bottomStop)) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                goToPosition(TransportPosition.COLLECT.ticks);
            } else {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-opMode.gamepad2.left_trigger);
            }
            headedUp = false;
            manualSlide = true;
            isHandled = false;

        } else {  // not manual control
            if (manualSlide) {  // just finished manual control, so stay here
                goToPosition(arm.getCurrentPosition());
                manualSlide = false;
            }

//            if (!arm.isBusy()){
//                headedUp = false;
//                //possible problem?
//            }

            if (opMode.gamepad2.a) {
                if (levelOne == 0) {  // prevent flip-down if already at 'a'
                    levelOne = 1;
                    goToPosition(TransportPosition.LEVEL1_PART_1.ticks);
                    setHeadedUp();
                }

            } else if (opMode.gamepad2.b && !isHandled) {
                goToPosition(TransportPosition.LEVEL2.ticks);
                isHandled = true;
                setHeadedUp();

            } else if (opMode.gamepad2.y && !isHandled) {
                goToPosition(TransportPosition.LEVEL3.ticks);
                isHandled = true;
                setHeadedUp();

            } else if (opMode.gamepad2.right_stick_button && !isHandled) {
                goToPosition(TransportPosition.MAX.ticks);
                isHandled = true;
                setHeadedUp();

            } else if (isTriggered(bottomStop) && !headedUp) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                goToPosition(TransportPosition.COLLECT.ticks);

                if (!shockAbsorberEngaged) {
                    shockAbsorber.setPosition(0);
//                    opMode.sleep(SLEEP_TIME);
                    shockAbsorberEngaged = true;
                }
                isHandled = false;

            } else if (opMode.gamepad2.x && !isHandled) {
                goToPosition(TransportPosition.COLLECT.ticks);
                isHandled = true;
                setHeadedUp();

            } else if (levelOne == 1 && !arm.isBusy()) {
                levelOne = 2;
                goToPosition(TransportPosition.LEVEL1_PART_2.ticks);
                headedUp = false;

            }else {
                if(isTriggered(topStop)){
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm.setPower(0);
                } else if (levelOne == 2 && !arm.isBusy()) {
                    levelOne = 3;
                    headedUp = false;
                } else if (!opMode.gamepad2.b && !opMode.gamepad2.y && !opMode.gamepad2.x && !opMode.gamepad2.right_stick_button) {
                    isHandled = false;
                }
            }
        }

        opMode.telemetry.addData("arm encoder", arm.getCurrentPosition());
        opMode.telemetry.addData("is Handled", isHandled);
        opMode.telemetry.addData("arm is busy", arm.isBusy());
        opMode.telemetry.addData("headed Up", headedUp);
        opMode.telemetry.addData("shock engaged", shockAbsorberEngaged);
        opMode.telemetry.addData("levelOne", levelOne);
        opMode.telemetry.addData("bottom magnet sensor", isTriggered(bottomStop));
        opMode.telemetry.addData("top magnet sensor", isTriggered(topStop));

        transporter.controlFlip();
    }

    private void goToPosition(int position) {
        if (isTriggered(topStop)) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(0);

        } else if (levelOne == 3 && opMode.gamepad2.x) {  // decided not to dump at level 1
            levelOne = 0;  // allow flip before doing down, but not going up
        } else {
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            setHeadedUp();

            if (headedUp) {
                if (shockAbsorberEngaged) { // lift shock absorber before starting up
                    shockAbsorber.setPosition(1);
                    shockAbsorberEngaged = false;
                }
                arm.setPower(UP_POWER);
            } else {
                arm.setPower(DOWN_POWER);
            }
        }
    }

    public void autoCollectPosition(double duckPosition) {

        if (duckPosition == 1) {
            arm.setTargetPosition(TransportPosition.LEVEL1_PART_1.ticks);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(UP_POWER);
            setHeadedUp();
            while (arm.isBusy()) {

            }
        }
        arm.setTargetPosition(TransportPosition.COLLECT.ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setHeadedUp();
        if (headedUp){
            if(shockAbsorberEngaged){
                shockAbsorber.setPosition(1);
                shockAbsorberEngaged = false;
            }
            arm.setPower(UP_POWER);
        }else {
            arm.setPower(DOWN_POWER);
        }
        while(opMode.opModeIsActive() && arm.isBusy()){
            transporter.controlFlip();
        }
        if (isTriggered(bottomStop)) {
            shockAbsorber.setPosition(0);
//                    opMode.sleep(SLEEP_TIME);
            shockAbsorberEngaged = true;
        }
    }

    public void goToPositionAuto(int duckPosition) {
        int position = TransportPosition.LEVEL3.ticks;

        if(duckPosition == 1){
            position = TransportPosition.LEVEL1_PART_1.ticks;
        }else if(duckPosition == 2) {
            position = TransportPosition.LEVEL2.ticks;
        }


        if (isTriggered(topStop)) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(0);

//        } else if (levelOne == 3 && opMode.gamepad2.x) {  // decided not to dump at level 1
//            levelOne = 0;  // allow flip before doing down, but not going up
        } else if(duckPosition == 1){
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setHeadedUp();
            if (headedUp){
                if(shockAbsorberEngaged){
                    shockAbsorber.setPosition(1);
                    shockAbsorberEngaged = false;
                }
                arm.setPower(UP_POWER);
            }else {
                arm.setPower(DOWN_POWER);
            }
            while(opMode.opModeIsActive() && arm.isBusy()){
                transporter.controlFlip();
            }
            arm.setTargetPosition(TransportPosition.LEVEL1_PART_2.ticks);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opMode.opModeIsActive() && arm.isBusy()){
            }
            transporter.scoreFreight();
            opMode.sleep(1500);

        } else if(duckPosition == 2 || duckPosition == 3){
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setHeadedUp();

            if (headedUp) {
                if (shockAbsorberEngaged) { // lift shock absorber before starting up
                    shockAbsorber.setPosition(1);
                    shockAbsorberEngaged = false;
                }
                arm.setPower(UP_POWER);
            } else {
                arm.setPower(DOWN_POWER);
            }
            while (opMode.opModeIsActive() && arm.isBusy()){
                transporter.controlFlip();
                opMode.telemetry.addData("slide moving - encoder count", arm.getCurrentPosition());
                opMode.telemetry.update();
            }
            transporter.scoreFreight();
            opMode.sleep(1500);
/*            goToPosition(TransportPosition.COLLECT.ticks);*/
        }
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

    public void setLevelOne(int level) {
        levelOne = level;
    }

    public boolean getHeadedUp(){
        return headedUp;
    }
    public void setHeadedUp(){
        if (arm.getTargetPosition() > arm.getCurrentPosition() || opMode.gamepad2.right_trigger > .1){
            headedUp = true;
        }else{
            headedUp = false;
        }
    }
}