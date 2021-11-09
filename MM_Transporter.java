package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
// Any additional import statements will go here

public class MM_Transporter {
    // this gives us access to all opMode information
    private LinearOpMode opMode;

    private Servo transport = null;

    private final double TRANSPORT_FLIP = 1000;

    // Constructor
    public MM_Transporter(LinearOpMode opMode) {
        this.opMode = opMode;

        transport = opMode.hardwareMap.get(Servo.class, "transport");
        transport.setPosition(1);  // collect position
    }

    public void controlFlip() {
        if (opMode.gamepad2.dpad_up){ // deposit freight
            transport.setPosition(0);
        }
        else if (opMode.gamepad2.dpad_down){
            transport.setPosition(1); // over-ride for collect position
        }
        else if (((MM_TeleOp) opMode).robot.slide.getSlidePosition() > TRANSPORT_FLIP) {//going up
            transport.setPosition(.5);
        } else if (((MM_TeleOp) opMode).robot.slide.getSlidePosition() < TRANSPORT_FLIP) {//going down
            transport.setPosition(1);
        }
    }
}
