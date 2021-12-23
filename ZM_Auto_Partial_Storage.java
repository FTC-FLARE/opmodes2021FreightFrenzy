package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ZM_Auto_Partial_Storage", group="MM")
@Disabled
public class ZM_Auto_Partial_Storage extends LinearOpMode {
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

        robot.drivetrain.driveForwardInches(24, 4);
        robot.vuforia.deactivateTargets();
        robot.vuforia.deactivateTfod();
    }
}
