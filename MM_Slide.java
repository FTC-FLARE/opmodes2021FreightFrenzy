package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Slide {

    enum TransportPosition {
        COLLECT(0),
        LEVEL1_PART_1(1100),
        LEVEL1_PART_2(800),
        LEVEL2(1800),
        LEVEL3(2600),
        MAX(3150);

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
    private double DOWN_POWER = .6;
    private int SLEEP_TIME = 1500;

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
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
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
//                opMode.sleep(SLEEP_TIME);
                shockAbsorberEngaged = false;
            }
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (arm.getCurrentPosition() < TransportPosition.MAX.ticks) {
                arm.setPower(opMode.gamepad2.right_trigger);
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

            if (!arm.isBusy()){
                headedUp = false;
            }

            if (opMode.gamepad2.a) {
                headedUp = true;
                if (levelOne == 0) {  // prevent flip-down if already at 'a'
                    levelOne = 1;
                    goToPosition(TransportPosition.LEVEL1_PART_1.ticks);
                }

            } else if (opMode.gamepad2.b && !isHandled) {
                headedUp = true;
                goToPosition(TransportPosition.LEVEL2.ticks);
                isHandled = true;

            } else if (opMode.gamepad2.y && !isHandled) {
                headedUp = true;
                goToPosition(TransportPosition.LEVEL3.ticks);
                isHandled = true;

            } else if (opMode.gamepad2.right_stick_button && !isHandled) {
                headedUp = true;
                goToPosition(TransportPosition.MAX.ticks);
                isHandled = true;

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

            } else if (levelOne == 1 && !arm.isBusy()) {
                levelOne = 2;
                goToPosition(TransportPosition.LEVEL1_PART_2.ticks);

            } else if (levelOne == 2 && !arm.isBusy()) {
                levelOne = 3;
            } else {
                isHandled = false;
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

            if (headedUp) {
                if (shockAbsorberEngaged) { // lift shock absorber before starting up
                    shockAbsorber.setPosition(1);
//                    opMode.sleep(SLEEP_TIME);
                    shockAbsorberEngaged = false;
                }
                arm.setPower(UP_POWER);
            } else {
                arm.setPower(DOWN_POWER);
            }
        }
    }

    //    public void goHome() {
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        // ****** This needs to become state logic *****
//
//        while (opMode.opModeIsActive() && !isTriggered(bottomStop)) {
//            // ******************** MUST hold dpad_down *************************************
//            ((MM_TeleOp) opMode).robot.transporter.controlFlip();
//        }
//        arm.setPower(.5);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(COLLECT);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public boolean headedUp() {
//        if (arm.getTargetPosition() > arm.getCurrentPosition() || opMode.gamepad2.right_trigger > .1) {
//            return true;
//        }
//        return false;
//    }

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

    public boolean isHeadedUp(){
        return headedUp;
    }
}