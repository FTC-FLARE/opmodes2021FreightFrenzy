package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_Robot {
    private MM_OpMode opMode;

    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Slide slide;
    public MM_Ducker ducker;
    public MM_Vuforia vuforia;

    // Constructor
    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        slide = new MM_Slide(opMode);
        ducker = new MM_Ducker(opMode);
        if (opMode.getClass() == MM_TeleOp.class) {
            drivetrain.initOdometryServos(1);
        } else{
            drivetrain.initOdometryServos(0);
            vuforia = new MM_Vuforia(opMode);
            drivetrain.initializeGyro();
        }
    }
}