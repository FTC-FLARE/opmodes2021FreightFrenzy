package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="MM_Auto", group="MM")
public class MM_Auto extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    static final int RED = 1;
    static final int BLUE = 2;
    static final int WAREHOUSE = 1;
    static final int STORAGE = 2;
    static final int OOTW = 1;
    static final int PARK = 2;
    private int alliance = RED;
    private int startingPosition = WAREHOUSE;
    private int finishPosition = OOTW;
    private int sleepTime = 0;
    private boolean spinDucker = false;
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
            }else if(gamepad1.y && spinDucker == false && !isHandled){
                spinDucker = true;
                isHandled = true;
            }else if(gamepad1.y && spinDucker == true && !isHandled){
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
            telemetry.addData(finishPosition == OOTW ? "get out of the way" : "park", spinDucker == true ? "spin ducker" : "");
            telemetry.update();

        }
        int duckLocation = robot.vuforia.findDuckPosition();

        if(alliance == RED){
            if(startingPosition == WAREHOUSE){
                robot.drivetrain.driveToHub("Blue Storage", duckLocation);
                robot.slide.goToPositionAuto(duckLocation);
                robot.slide.autoCollectPosition(duckLocation);
                robot.drivetrain.outOfTheWay(false);
            }else if(startingPosition == STORAGE){
                robot.drivetrain.driveToHub("Blue Warehouse", duckLocation);
                robot.slide.goToPositionAuto(duckLocation);
                robot.slide.autoCollectPosition(duckLocation);
                robot.drivetrain.storagePark(false, duckLocation, true);
            }
        }else if(alliance == BLUE){
            if(startingPosition == WAREHOUSE){
                robot.drivetrain.driveToHub("Blue Warehouse", duckLocation);
                robot.slide.goToPositionAuto(duckLocation);
                robot.slide.autoCollectPosition(duckLocation);
                robot.drivetrain.outOfTheWay(true);
            }else if(startingPosition == STORAGE){
                robot.drivetrain.driveToHub("Blue Storage", duckLocation);
                robot.slide.goToPositionAuto(duckLocation);
                robot.slide.autoCollectPosition(duckLocation);
                robot.drivetrain.storagePark(true, duckLocation, true);
            }
        }

        robot.vuforia.deactivateTargets();
        robot.vuforia.deactivateTfod();
    }

    private void initializeOpmode() {
        telemetry.addData("Status", "Wait for initialization");
        telemetry.update();

        robot.autoInit();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
