package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TEST_DEAD_ENCODER", group="MM")
//@Disabled
public class TEST_Dead_Encoder extends MM_OpMode {
    private MM_Robot robot = new MM_Robot(this);
    private ElapsedTime runtime = new ElapsedTime();
    private double driveInches = 48;
    private double secondDriveInches = 12;
    private boolean isHandled = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();

        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            if(gamepad1.dpad_up && !isHandled){
                 driveInches += 1;
                isHandled = true;
            }else if(gamepad1.dpad_down && !isHandled){
                driveInches -= 1;
                isHandled = true;
            }else if(gamepad1.dpad_right && !isHandled){
                secondDriveInches += 1;
                isHandled = true;
            }else if(gamepad1.dpad_left && !isHandled){
                secondDriveInches -= 1;
                isHandled = true;
            } else if(!gamepad1.a && !gamepad1.b && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.y){
                isHandled = false;
            }
            telemetry.addData("Inches", driveInches);
            telemetry.addData("Second Inches", secondDriveInches);
            telemetry.update();
        }
        runtime.reset();

        robot.drivetrain.driveForwardInches(driveInches, 7);
        sleep(2000);
        robot.drivetrain.driveForwardInches(secondDriveInches, 7);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        robot.vuforia.deactivateTargets();
        robot.vuforia.deactivateTfod();
    }
}
