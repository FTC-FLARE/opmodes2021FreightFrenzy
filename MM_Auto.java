package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MM_Auto", group="MM")
//@Disabled
public class MM_Auto extends LinearOpMode {
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

        int duckLocation = robot.vuforia.findDuckPosition();
        robot.drivetrain.deliveryDrive(duckLocation);
        robot.drivetrain.strafeRightInches(23, 4);
/*
        robot.drivetrain.rotate(178, 4);
*/
/*
        robot.slide.goToPositionAuto(duckLocation);
        robot.transporter.scoreFreight();
*/


        //robot.ducker.DuckerAuto(2);`

        /*robot.drivetrain.diagonalDriveInches(12, 12, 7);
        robot.drivetrain.diagonalDriveInches(-12, -12, 7);
        robot.drivetrain.diagonalDriveInches(-12, 12, 7);
        robot.drivetrain.diagonalDriveInches(12, -12, 7);*/

           if (robot.vuforia.targetFound()) {
               telemetry.addLine("Target is Found");

               telemetry.addData("X", "position (%.2f)", robot.vuforia.getX());
               telemetry.addData("Y", "position (%.2f)", robot.vuforia.getY());
               telemetry.addData("Heading", "(%.2f) degrees", robot.vuforia.getHeading());
            }
           else {
               telemetry.addLine("No target found.");
           }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        robot.vuforia.deactivateTargets();
        robot.vuforia.deactivateTfod();
    }
}
