package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="MM_Auto", group="Test")
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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        robot.ducker.DuckerAuto(2);

        robot.drivetrain.diagonalDriveInches(12, 12, 7);
        robot.drivetrain.diagonalDriveInches(-12, -12, 7);
        robot.drivetrain.diagonalDriveInches(-12, 12, 7);
        robot.drivetrain.diagonalDriveInches(12, -12, 7);


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
    }
}
