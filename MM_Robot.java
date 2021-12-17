package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MM_Robot {
    private LinearOpMode opMode;

    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Slide slide;
    public MM_Ducker ducker;
    public MM_Vuforia vuforia;

    // Constructor
    public MM_Robot(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void autoInit(){
        init();
        vuforia = new MM_Vuforia(opMode);
        drivetrain.initializeGyro();
        drivetrain.setOdometryServos(1);//set to down position
    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        slide = new MM_Slide(opMode);
        ducker = new MM_Ducker(opMode);
    }
}
