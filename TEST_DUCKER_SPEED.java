package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="DUCKER TEST", group="MM")
public class TEST_DUCKER_SPEED extends MM_OpMode {
    public MM_Robot robot = new MM_Robot(this);
    private boolean isHandled;
    private double minPower = 0.4;
    private double maxPower = 0.75;
    private double rampInterval = 0.01;
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

            if(gamepad1.dpad_up && !isHandled){
                minPower += 0.01;
                isHandled = true;
            }else if(gamepad1.dpad_down && !isHandled){
                minPower -= 0.01;
                isHandled = true;
            }else if(gamepad1.dpad_right && !isHandled){
                maxPower += 0.01;
                isHandled = true;
            }else if(gamepad1.dpad_left && !isHandled){
                maxPower -= 0.01;
                isHandled = true;
            } else if (gamepad1.a && !isHandled) {
                rampInterval += 0.001;
                isHandled = true;
            } else if (gamepad1.b && !isHandled) {
                rampInterval -= 0.001;
                isHandled = true;
            } else if (!gamepad1.b && !gamepad1.a && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_down && !gamepad1.dpad_up && isHandled) {
                isHandled = false;
            }
            telemetry.addLine("press 'a' to increase rampInterval");
            telemetry.addLine("press 'b' to decrease rampInterval");
            telemetry.addLine("press d-pad up or down to change minPower");
            telemetry.addLine("press d-pad left or right to change maxPower");
            telemetry.addData("minPower", minPower);
            telemetry.addData("maxPower", maxPower);
            telemetry.addData("rampInterval", rampInterval);
            telemetry.update();

            robot.ducker.manualSpinTEST(minPower, maxPower, rampInterval);

        }
    }
}
