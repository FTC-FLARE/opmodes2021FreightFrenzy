package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// Any additional import statements will go here

public class MM_Transporter {
    // this gives us access to all opMode information
    private LinearOpMode opMode;
    private MM_Slide slide;

    private Servo transport = null;
    private DistanceSensor transportDown = null;
    private DistanceSensor transportUp = null;

    private final double TRANSPORT_FLIP = 1550;
    private final double COLLECT_POSITION = 1;  // .7 works, but > .7 goes to 1 - why?
    private final double FINISH_COLLECT_POSITION = .7;
    private final double CARRY_POSITION = .4;
    private final double SCORE_POSITION = 0;

    // Constructor
    public MM_Transporter(LinearOpMode opMode, MM_Slide slide) {
        this.opMode = opMode;
        this.slide = slide;

        transport = opMode.hardwareMap.get(Servo.class, "transport");
//        transport.scaleRange(0, .8);
        transport.setPosition(COLLECT_POSITION);

        transportDown = opMode.hardwareMap.get(DistanceSensor.class, "transportDown");
        transportUp = opMode.hardwareMap.get(DistanceSensor.class, "transportUp");
    }

    public void controlFlip() {
        if (opMode.gamepad2.dpad_up) { // deposit freight
            transport.setPosition(SCORE_POSITION);
            ((MM_TeleOp) opMode).robot.slide.setLevelOne(0);
        } else if (opMode.gamepad2.dpad_down) {
            transport.setPosition(COLLECT_POSITION); // over-ride to collect position

        } else if (slide.getSlidePosition() > TRANSPORT_FLIP // cont'd
                || slide.getLevelOne() > 1  // cont'd
                || (transportUp.getDistance(DistanceUnit.CM) < 5 && (slide.getHeadedUp()))) {
            transport.setPosition(CARRY_POSITION);

        } else if (slide.getLevelOne() < 2   // cont'd
                && (transportDown.getDistance(DistanceUnit.CM) < 7 || (slide.getSlidePosition() < TRANSPORT_FLIP))) {
            transport.setPosition(COLLECT_POSITION);
        }

        opMode.telemetry.addData("transport up", transportUp.getDistance(DistanceUnit.CM));
        opMode.telemetry.addData("transport down", transportDown.getDistance(DistanceUnit.CM));
    }
}
