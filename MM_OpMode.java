package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public abstract class MM_OpMode extends LinearOpMode {

    static double DRIVE_P_COEFFICIENT = 0.000095;

    public MM_Robot robot = new MM_Robot(this);
    public MM_P_Controller pTurnController = new MM_P_Controller(this, 2, .01);
    public MM_P_Controller flMotorController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT);
    public MM_P_Controller frMotorController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT);
    public MM_P_Controller blMotorController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT);
    public MM_P_Controller brMotorController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT);
//    @Override
//    public void runOpMode() {
//        telemetry.addData("Status", "Initializing... Please wait");
//        telemetry.update();
//        robot.init();
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            robot.ducker.manualSpin();
//            robot.drivetrain.driveWithSticks();
//            robot.slide.runSlide();
//            robot.collector.runCollector();
//
//            telemetry.update();
//        }
//    }
}
