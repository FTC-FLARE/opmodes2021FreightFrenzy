package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MM_Auto", group="MM")
public class MM_Auto extends MM_OpMode {
    private MM_Robot robot = new MM_Robot(this);

    private boolean isHandled = false;
    private boolean xIsPressed = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeOpmode();
        while (!isStarted() && !isStopRequested()){
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
            }else if(gamepad1.b && finishPosition == CSP && !isHandled){
                finishPosition = PARK;
                isHandled = true;
            }else if(gamepad1.b && finishPosition == PARK && !isHandled){
                finishPosition = CSP;
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
            }else if (gamepad1.dpad_right && !isHandled) {
                distanceToCollect += 1;
                isHandled = true;
            }else if (gamepad1.dpad_left && !isHandled) {
                distanceToCollect -= 1;
                isHandled = true;
            }else if (gamepad1.x && !isHandled) {
                telemetry.addLine("Initializing Gyro");
                telemetry.update();
                robot.drivetrain.initializeGyroAndEncoders();
                xIsPressed = true;
            } else if(!gamepad1.a && !gamepad1.b && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.y && !gamepad1.x){
                isHandled = false;
            }
            telemetry.addLine("right or left bumper to change alliance");
            telemetry.addLine("press 'a' to change starting position");
            telemetry.addLine("press 'b' to change finish position");
            telemetry.addLine("press d-pad up or down to change sleep time");
            telemetry.addLine("press d-pad right or left to change collect distance");
            telemetry.addLine("press 'y' to turn ducker on or off");
            if (!xIsPressed) {
                telemetry.addLine("************NEED TO FINISH INITIALIZATION************");
                telemetry.addLine("press 'x' after your robot is positioned correctly");
            } else {
                telemetry.addLine("robot is fully initialized!");
            }
            telemetry.addLine();
            telemetry.addData("sleep time", sleepTime);
            telemetry.addData("collect distance", distanceToCollect);
            telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == WAREHOUSE ? "Warehouse" : "Storage");
            telemetry.addData(finishPosition == CSP ? "collect, score, and Park" : "park", spinDucker ? "spin ducker" : "");
            telemetry.update();
        }
        //*************************************** DRIVER HIT PLAY **************************************************
        if (!xIsPressed) {
            robot.drivetrain.initializeGyroAndEncoders();
        }
        scorePosition = robot.vuforia.findDuckPosition();

        sleep(sleepTime); //driver-selected sleep time

        robot.driveToHub();
        scoreFreight();

        if(startingPosition == WAREHOUSE){
            robot.warehouseCollect();
            robot.ScoreAndPark();
        }else if(startingPosition == STORAGE){
            //make sure to run down slide first
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

    private void scoreFreight(){
        robot.slide.transporter.scoreFreight();
        if (startingPosition == WAREHOUSE){
            sleep(1500);
        } else {
            runtime.reset();
            robot.vuforia.switchCameraMode(VUFORIA);
            if (runtime.seconds() < 1.5) {
                double timeDifference = 1.5 - runtime.seconds();
                runtime.reset();
                while (runtime.seconds() < timeDifference){
                }
            }
            telemetry.addData("switched Camera", true);
        }
        robot.slide.transporter.carryFreight();
    }
}
