package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group="MM")
public class MM_TeleOp extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    @Override
    public void runOpMode() {
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            robot.drivetrain.driveWithSticks();
            robot.slide.runSlide();

            telemetry.update();
        }
    }
}
