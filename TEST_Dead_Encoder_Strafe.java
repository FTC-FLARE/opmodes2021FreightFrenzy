package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TEST_DEAD_ENCODER_STRAFE", group="MM")
//@Disabled
public class TEST_Dead_Encoder_Strafe extends MM_OpMode {
    private MM_Robot robot = new MM_Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();

        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.drivetrain.driveForwardToPosition(12, 7);
        robot.drivetrain.pRotateDegrees(90);
        robot.drivetrain.driveForwardToPosition(25, 7);
        robot.drivetrain.pRotateDegrees(180);
        robot.drivetrain.driveForwardToPosition(-5, 4);
        robot.slide.goToPositionAuto(3);

        robot.drivetrain.pRotateDegrees(-90);
        robot.drivetrain.driveForwardToPosition(35, 6);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        robot.vuforia.deactivateTargets();
        robot.vuforia.deactivateTfod();
    }
}
