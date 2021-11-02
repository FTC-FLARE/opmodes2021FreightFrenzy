package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.localfiles.FFVuforia;




@TeleOp(name="TeleOp", group="MM")
//@Disabled
public class MM_TeleOp extends LinearOpMode {
    private MM_Drivetrain robot = new MM_Drivetrain(this);
    private ElapsedTime runtime = new ElapsedTime();
    private FFVuforia vuforia = new FFVuforia(this);
    @Override
    public void runOpMode() {
        vuforia.vuforiaInit();
        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            robot.driveWithSticks();

            /*if (gamepad1.y & !isHandled) {

                if (isOpen) {
                    robot.setArmServo(1);
                }
                else {
                    robot.setArmServo(0);
                }
                isOpen = !isOpen;
                isHandled = true;
            }


            else if (!gamepad1.y) {
                isHandled = false;
            }*/

           if (vuforia.targetFound()) {
               telemetry.addLine("Target is Found");

               telemetry.addData("X", "position (%.2f)", vuforia.getX());
               telemetry.addData("Y", "position (%.2f)", vuforia.getY());
               telemetry.addData("Heading", "(%.2f) degrees", vuforia.getHeading());
            }
           else {
               telemetry.addLine("No target found.");
           }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
        vuforia.deactivateTargets();
    }
}
