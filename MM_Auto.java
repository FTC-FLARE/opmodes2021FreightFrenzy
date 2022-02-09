package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="MM_Auto", group="MM")
public class MM_Auto extends MM_OpMode {
    private MM_Robot robot = new MM_Robot(this);

    private boolean isHandled = false;

    @Override
    public void runOpMode() {
        initializeOpmode();
        while (!isStarted()){
            if(gamepad1.right_bumper && alliance == RED && !isHandled){
                alliance = BLUE;
                isHandled = true;
            }else if(gamepad1.left_bumper && alliance == BLUE && !isHandled){
                alliance = RED;
                isHandled = true;
            }else if(gamepad1.a && startingPosition == WAREHOUSE && !isHandled){
                startingPosition = STORAGE;
                isHandled = true;
            }else if(gamepad1.a && startingPosition == STORAGE && !isHandled){
                startingPosition = WAREHOUSE;
                isHandled = true;
            }else if(gamepad1.b && finishPosition == OOTW && !isHandled){
                finishPosition = PARK;
                isHandled = true;
            }else if(gamepad1.b && finishPosition == PARK && !isHandled){
                finishPosition = OOTW;
                isHandled = true;
            }else if(gamepad1.y && !spinDucker && !isHandled){
                spinDucker = true;
                isHandled = true;
            }else if(gamepad1.y && spinDucker && !isHandled){
                spinDucker = false;
                isHandled = true;
            }else if(sleepTime >= 15000){
                sleepTime = 0;
            }else if(gamepad1.dpad_up && !isHandled){
                sleepTime += 1000;
                isHandled = true;
            }else if(gamepad1.dpad_down && sleepTime > 0 && !isHandled){
                sleepTime -= 1000;
                isHandled = true;
            }else if(!gamepad1.a && !gamepad1.b && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.y){
                isHandled = false;
            }
            telemetry.addLine("right or left bumper to change alliance");
            telemetry.addLine("press 'a' to change starting position");
            telemetry.addLine("press 'b' to change finish position");
            telemetry.addLine("press d-pad up or down to change sleep time");
            telemetry.addLine("press 'y' to turn ducker on or off");
            telemetry.addLine();
            telemetry.addData("sleep time", sleepTime);
            telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == WAREHOUSE ? "Warehouse" : "Storage");
            telemetry.addData(finishPosition == OOTW ? "get out of the way" : "park", spinDucker ? "spin ducker" : "");
            telemetry.update();
        }
        //*************************************** DRIVER HIT PLAY **************************************************
        scorePosition = robot.vuforia.findDuckPosition();

        sleep(sleepTime); //driver-selected sleep time

        robot.scoreOnHub();

        if(startingPosition == WAREHOUSE){
            robot.warehousePark();
            robot.collector.autoCollect();
            scorePosition = 3;
        }else if(startingPosition == STORAGE){
            if (spinDucker) {
                robot.goDuck();
            }
            robot.storagePark();
        }

        robot.vuforia.deactivateTargets();
        robot.vuforia.deactivateTfod();
    }

    private void initializeOpmode() {
        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();

        robot.init();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
