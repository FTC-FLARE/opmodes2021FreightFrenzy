package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Slide {

    enum TransportPosition {
        COLLECT(0),
        LEVEL1_PART_1(1875),
        LEVEL1_PART_2(1275),
        LEVEL2(2780),
        LEVEL3(4150),
        MAX(4670);

        public final int ticks;

        private TransportPosition(int ticks) {
            this.ticks = ticks;
        }
    }

    private MM_OpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    public MM_Transporter transporter = null;
    //    private DigitalChannel bottomStop = null;
    private AnalogInput bottomStop = null;
    private DigitalChannel topStop = null;
    private DcMotor arm = null;
    private Servo shockAbsorber = null;

    private TransportPosition selectedPosition = TransportPosition.COLLECT;

    private int level1Progress = NOT_LEVEL_1;
    private boolean manualSlide = false;
    private boolean isHandled = false;
    private boolean isHandledTime = false;
    private boolean headedUp = false;
    private boolean shockAbsorberEngaged = true;
    public boolean level1Safety = false;

    private final double UP_POWER = 1;
    private final double DOWN_POWER = 0.65;
    private final double AUTO_DOWN_POWER = -0.60;
    private final double AUTO_FIX_POWER = 0.1;

    static final int NOT_LEVEL_1 = 0;
    static final int MOVING_TO_FLIP_POSITION = 1;
    static final int MOVING_TO_SCORE_POSITION = 2;
    static final int CHILLIN_AT_LEVEL_1 = 3;

    public MM_Slide(MM_OpMode opMode) {
        this.opMode = opMode;

        transporter = new MM_Transporter(opMode, this);
        init();
    }

    public void runSlide() {
        if (opMode.gamepad2.right_trigger > .1) { // slide request up
            engageShock(false);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (arm.getCurrentPosition() < TransportPosition.MAX.ticks && !isTriggered(topStop)) {
                arm.setPower(opMode.gamepad2.right_trigger);
            } else {
                arm.setPower(0);
            }
            headedUp = true;
            manualSlide = true;
            isHandled = false;

        } else if (opMode.gamepad2.left_trigger > .1) { // slide request down
            if (isTriggeredMRtouch(bottomStop)) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setSlideTarget(TransportPosition.COLLECT.ticks);
            } else {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-opMode.gamepad2.left_trigger);
            }
            headedUp = false;
            manualSlide = true;
            isHandled = false;

        } else {  // not manual control
            if (manualSlide) {  // just finished manual control, so stay here
                setSlideTarget(arm.getCurrentPosition());
                manualSlide = false;
            }

            if (opMode.gamepad2.a) {
                // prevent flip-down if working on 'a'
                if (level1Progress == NOT_LEVEL_1) {
                    level1Progress = MOVING_TO_FLIP_POSITION;
                    setSlideTarget(TransportPosition.LEVEL1_PART_1.ticks);
                    setHeadedUp();
                }
            } else if (!isHandled && (opMode.gamepad2.b || opMode.gamepad2.y || opMode.gamepad2.right_stick_button)) {
                if (opMode.gamepad2.b) {
                    setSlideTarget(TransportPosition.LEVEL2.ticks);
                } else if (opMode.gamepad2.y) {
                    setSlideTarget(TransportPosition.LEVEL3.ticks);
                } else if (opMode.gamepad2.right_stick_button) {
                    setSlideTarget(TransportPosition.MAX.ticks);
                }
                isHandled = true;
                setHeadedUp();

            } else if (isTriggeredMRtouch(bottomStop) && !headedUp) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setSlideTarget(TransportPosition.COLLECT.ticks);

                engageShock(true);
                isHandled = false;

            } else if (opMode.gamepad2.x && !isHandled) {
                if (level1Progress == CHILLIN_AT_LEVEL_1) {
                    level1Progress = MOVING_TO_FLIP_POSITION;
                    setSlideTarget(TransportPosition.LEVEL1_PART_1.ticks);
                    setHeadedUp();
                    transporter.carryFreight();
                    level1Safety = true;
                    isHandled = true;
                } else if (!level1Safety || level1Progress == NOT_LEVEL_1) {
                    setSlideTarget(TransportPosition.COLLECT.ticks);
                    isHandled = true;
                    setHeadedUp();
                }
            } else if (level1Progress == MOVING_TO_FLIP_POSITION && !arm.isBusy()) {
                level1Progress = MOVING_TO_SCORE_POSITION;
                setSlideTarget(TransportPosition.LEVEL1_PART_2.ticks);
                headedUp = false;
            } else if (level1Safety) {
                if (arm.getCurrentPosition() > 1450) {
                    level1Safety = false;
                    level1Progress = NOT_LEVEL_1;
                    setSlideTarget(TransportPosition.COLLECT.ticks);
                    isHandled = true;
                    setHeadedUp();
                }
            } else {
                if (isTriggered(topStop)) {
                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm.setPower(0);
                } else if (level1Progress == MOVING_TO_SCORE_POSITION && !arm.isBusy()) {
                    level1Progress = CHILLIN_AT_LEVEL_1;
                    headedUp = false;
                    level1Safety = true;
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
        opMode.telemetry.addData("levelOne", level1Progress);
        opMode.telemetry.addData("bottom magnet sensor", bottomStop.getVoltage());
        opMode.telemetry.addData("top magnet sensor", isTriggered(topStop));

        transporter.controlFlip();
    }

    private void setSlideTarget(int position) {
        if (isTriggered(topStop)) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(0);

        } else if (level1Progress == CHILLIN_AT_LEVEL_1 && opMode.gamepad2.x) {  // decided not to dump at level 1
            level1Progress = NOT_LEVEL_1;  // allow flip before doing down, but not going up
        } else {
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setHeadedUp();

            if (headedUp) {
                engageShock(false);
                arm.setPower(UP_POWER);
            } else {
                arm.setPower(DOWN_POWER);
            }
        }
    }

    public boolean reachedPositionUp() {
        transporter.controlFlipAutoUp(false);
        if (transporter.seesBox || getSlidePosition() > 1775) {
            arm.setPower(0);
            return true;
        }
        return false;
    }

    public boolean reachedPositionUp2() {
        transporter.controlFlipAutoUp(false);
        if (getSlidePosition() > 2780) {
            arm.setPower(0);
            return true;
        }
        return false;
    }

    public boolean reachedPositionDown() {
        transporter.controlFlipAutoDown();
        if (isTriggeredMRtouch(bottomStop)) {
            if (!isHandledTime) {
                runtime.reset();
                isHandledTime = true;
            }
            if (runtime.seconds() > 0.00) {
                arm.setPower(0);
                engageShock(true);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                return true;
            }
        }
        return false;
    }

    public void fixPosition() {
        arm.setTargetPosition(TransportPosition.COLLECT.ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(Math.abs(AUTO_DOWN_POWER));
        while (arm.isBusy() && opMode.opModeIsActive()) {
            if (isTriggeredMRtouch(bottomStop)) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                engageShock(true);
            }
        }
    }

    public void fixSensor() {
        if (!isTriggeredMRtouch(bottomStop)) {
            arm.setPower(AUTO_FIX_POWER);
            runtime.reset();
            while (!isTriggeredMRtouch(bottomStop) && opMode.opModeIsActive() && runtime.seconds() < 1.5) {
            }
            arm.setPower(0);
            if (isTriggeredMRtouch(bottomStop)) {
                engageShock(true);
            }
        }
    }

    public void startRaising() {
        engageShock(false);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(UP_POWER);
    }

    public void startLowering() {
        if (opMode.scorePosition == 1) {
            moveToLevel1Part1();
        }
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(AUTO_DOWN_POWER);
    }

    public void runSlideToScore() {
        int armTarget = TransportPosition.LEVEL3.ticks;

        if (opMode.scorePosition == 1) {
            armTarget = TransportPosition.LEVEL1_PART_1.ticks;
        } else if (opMode.scorePosition == 2) {
            armTarget = TransportPosition.LEVEL2.ticks;
        }

        if (isTriggered(topStop)) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(0);

        } else {
            arm.setTargetPosition(armTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setHeadedUp();

            if (headedUp) {
                engageShock(false);
                arm.setPower(UP_POWER);
            } else {
                arm.setPower(DOWN_POWER);
            }

            while (opMode.opModeIsActive() && arm.isBusy()) {
                transporter.carryFreight();
                opMode.telemetry.addData("slide moving - encoder count", arm.getCurrentPosition());
                opMode.telemetry.update();
            }

            if (opMode.scorePosition == 1) {
                arm.setTargetPosition(TransportPosition.LEVEL1_PART_2.ticks);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while (opMode.opModeIsActive() && arm.isBusy()) {
                }
            }
        }
    }

    private void moveToLevel1Part1() { //only if you are going down
        arm.setTargetPosition(MM_Slide.TransportPosition.LEVEL1_PART_1.ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(UP_POWER);
        setHeadedUp();
        while (arm.isBusy() && opMode.opModeIsActive()) {
        }
    }

    public int getSlidePosition() {
        return arm.getCurrentPosition();
    }

    public boolean isTriggered(DigitalChannel sensor) {
        return !(sensor.getState());
    }

    public int getLevel1Progress() {
        return level1Progress;
    }

    public void setLevel1Progress(int target) {
        level1Progress = target;
    }

    public boolean isHeadedUp() {
        return headedUp;
    }

    public void setHeadedUp() {
        if (arm.getTargetPosition() > arm.getCurrentPosition() || opMode.gamepad2.right_trigger > .1) {
            headedUp = true;
        } else {
            headedUp = false;
        }
    }

    public boolean isTriggeredMRtouch(AnalogInput sensor) {
        double voltage = (sensor.getVoltage());
        if (voltage > 2) {
            return true;
        } else {
            return false;
        }
    }

    private void engageShock(boolean engage) {
        if (shockAbsorberEngaged != engage) {
            if (engage){
                shockAbsorber.setPosition(0);
            }else {
                shockAbsorber.setPosition(1);
            }
            shockAbsorberEngaged = engage;
        }
    }

    public void stop() {
        arm.setPower(0);
    }

    private void init() {
        arm = opMode.hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shockAbsorber = opMode.hardwareMap.get(Servo.class, "shockAbsorber");
        shockAbsorber.setPosition(0); // engage

//        bottomStop = opMode.hardwareMap.get(DigitalChannel.class, "bottomStop");//bottom limit switch on the slide
//        bottomStop.setMode(DigitalChannel.Mode.INPUT);
        topStop = opMode.hardwareMap.get(DigitalChannel.class, "topStop");//top limit switch on the slide
        bottomStop = opMode.hardwareMap.get(AnalogInput.class, "MR_touch");

    }
}
/*
    public void autoCollectPosition() {
        //needs cleaning
        if (opMode.scorePosition == 1) {
            moveToLevel1Part1();
        }
        arm.setTargetPosition(TransportPosition.COLLECT.ticks);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setHeadedUp();
        if (headedUp) {
            engageShock(false);
            arm.setPower(UP_POWER);
        } else {
            arm.setPower(DOWN_POWER);
        }
        while (opMode.opModeIsActive() && arm.isBusy()) {
            transporter.controlFlip();
        }
        if (isTriggeredMRtouch(bottomStop)) {
            engageShock(true);
        }
    }
*/
