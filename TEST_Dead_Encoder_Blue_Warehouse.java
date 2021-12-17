package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TEST_DEAD_ENCODER", group="MM")
//@Disabled
public class TEST_Dead_Encoder_Blue_Warehouse extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();

        robot.autoInit();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.drivetrain.driveForwardToPosition(24, 7);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        robot.vuforia.deactivateTargets();
        robot.vuforia.deactivateTfod();
    }
}
