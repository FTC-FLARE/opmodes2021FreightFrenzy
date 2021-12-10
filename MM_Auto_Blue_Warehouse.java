package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MM_Blue_Warehouse_Park", group="MM")
//@Disabled
public class MM_Auto_Blue_Warehouse extends LinearOpMode {
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

        int duckLocation = robot.vuforia.findDuckPosition();
        robot.drivetrain.driveToHub("Blue Warehouse", duckLocation);


        robot.slide.goToPositionAuto(duckLocation);

        robot.slide.autoCollectPosition(duckLocation);
        robot.drivetrain.storagePark(true, duckLocation, false);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        robot.vuforia.deactivateTargets();
        robot.vuforia.deactivateTfod();
    }
}
