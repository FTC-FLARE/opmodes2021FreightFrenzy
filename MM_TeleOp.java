package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="MM")
public class MM_TeleOp extends MM_OpMode {
    public MM_Robot robot = new MM_Robot(this);
    boolean isHandled = false;
    double rampInterval = 0.037;

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

            robot.ducker.manualSpin();
            robot.drivetrain.driveWithSticks();
            robot.slide.runSlide();
            robot.collector.runCollector();
/*            robot.capper.runCapper();*/
            telemetry.update();
        }
    }
}
