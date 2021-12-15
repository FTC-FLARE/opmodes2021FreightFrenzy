package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Transporter {
    // this gives us access to all opMode information
    private LinearOpMode opMode;
    private MM_Slide slide;

    private Servo transport = null;
    private DistanceSensor transportDown = null;
    private DistanceSensor transportUp = null;

    private final double TRANSPORT_FLIP = 1550;

    // .6 works for current servo (why?)- a change in servo may require a different setting
    private final double COLLECT_POSITION = .6;

    private final double CARRY_POSITION = .4;
    private final double SCORE_POSITION = 0;

    // Constructor
    public MM_Transporter(LinearOpMode opMode, MM_Slide slide) {
        this.opMode = opMode;
        this.slide = slide;

        transport = opMode.hardwareMap.get(Servo.class, "transport");
        transport.setPosition(COLLECT_POSITION);

        transportDown = opMode.hardwareMap.get(DistanceSensor.class, "transportDown");
        transportUp = opMode.hardwareMap.get(DistanceSensor.class, "transportUp");
    }

    public void controlFlip() {
        if (opMode.gamepad2.dpad_up) { // deposit freight
            transport.setPosition(SCORE_POSITION);
            slide.setLevel1Progress(MM_Slide.NOT_LEVEL_1);
        } else if (opMode.gamepad2.dpad_down) {
            transport.setPosition(COLLECT_POSITION); // over-ride to collect position

        } else if (slide.getSlidePosition() > TRANSPORT_FLIP // cont'd
                || slide.getLevel1Progress() > MM_Slide.MOVING_TO_FLIP_POSITION  // cont'd
                || (seesBox(transportUp) && (slide.isHeadedUp()))) {
            transport.setPosition(CARRY_POSITION);

        } else if (slide.getLevel1Progress() < MM_Slide.MOVING_TO_SCORE_POSITION   // cont'd
                && (seesBox(transportDown) || (slide.getSlidePosition() < TRANSPORT_FLIP))) {
            transport.setPosition(COLLECT_POSITION);
        }

        opMode.telemetry.addData("transport up", transportUp.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("transport down", transportDown.getDistance(DistanceUnit.CM));
    }

    public void scoreFreight() {
        transport.setPosition(SCORE_POSITION);
    }

    public boolean seesBox(DistanceSensor distanceSensor) {

        double checkDistance = 5;
        if (distanceSensor == transportDown) {
            checkDistance = 7;
        }

        if (distanceSensor.getDistance(DistanceUnit.CM) < checkDistance) {
            return true;
        }
        return false;
    }
}

