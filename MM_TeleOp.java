package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="MM")
public class MM_TeleOp extends LinearOpMode {
    public MM_Robot robot = new MM_Robot(this);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing... Please wait");
        telemetry.update();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            robot.ducker.DuckManual();
            robot.drivetrain.driveWithSticks();
            robot.slide.runSlide();
            robot.collector.runCollector();

            telemetry.update();
        }
    }
}
