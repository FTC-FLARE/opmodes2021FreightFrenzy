package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class  MM_Capper {

    private LinearOpMode opMode;

    private CRServo primary = null;
    private Servo secondary = null;
    private DigitalChannel topCapperStop;

    public MM_Capper(LinearOpMode opMode){
        this.opMode = opMode;

        primary = opMode.hardwareMap.get(CRServo.class, "primaryCapper");
        secondary = opMode.hardwareMap.get(Servo.class,"secondaryCapper");
        topCapperStop = opMode.hardwareMap.get(DigitalChannel.class,"topCapperStop");
    }
    public void runCapper(){
        if(-opMode.gamepad2.right_stick_y > 0.1){
            secondary.setPosition(1);
        }else if(-opMode.gamepad2.right_stick_y < -0.1){
            secondary.setPosition(0);
        }

        if(!topCapperStop.getState() && -opMode.gamepad2.left_stick_y < 0.1) {
            primary.setPower(0);
        }else{
            primary.setPower(-opMode.gamepad2.left_stick_y / 2);
        }

        opMode.telemetry.addData("capper power", opMode.gamepad2.left_stick_y);

    }
}
