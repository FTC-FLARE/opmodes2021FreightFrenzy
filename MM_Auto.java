package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MM_Auto", group="MM")
public class MM_Auto extends MM_OpMode {
    private MM_Robot robot = new MM_Robot(this);

    private boolean isHandled = false;
    private boolean xIsPressed = false;
    private boolean useVuforia = false;
    private final double SCORE_TIME = 1.5;

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
                spinDucker = true;
            }else if(gamepad1.a && startingPosition == STORAGE && !isHandled){
                startingPosition = WAREHOUSE;
                isHandled = true;
                spinDucker = false;
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
            }else if (gamepad1.right_trigger > 0.4 && !isHandled){
                smooshedBlocks = !smooshedBlocks;
                isHandled = true;
            } else if(!gamepad1.a && !gamepad1.b && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.y && !gamepad1.x && gamepad1.right_trigger < 0.4){
                isHandled = false;
            }

            if (!xIsPressed) {
                telemetry.addLine("press 'x' after your robot is positioned correctly");
                telemetry.addLine();
                telemetry.addLine("************************");
                telemetry.addLine("************************");
                telemetry.addLine("DONT START ROBOT");
                telemetry.addLine("************************");
                telemetry.addLine("************************");
                telemetry.addLine();
                telemetry.addLine("************************");
                telemetry.addLine("************************");
                telemetry.addLine("NEED TO FINISH INITIALIZATION");
                telemetry.addLine("************************");
                telemetry.addLine("************************");

            } else {
                telemetry.addLine("robot is fully initialized!");
                telemetry.addLine();
                telemetry.addLine("Right or left bumper to change alliance");
                telemetry.addLine("Press 'a' to change starting position");
                telemetry.addLine("Press 'b' to change finish position");
                telemetry.addLine("Press 'y' to turn ducker on or off");
                telemetry.addLine("Press d-pad up or down to change sleep time");
                if (startingPosition == WAREHOUSE) {
                    telemetry.addLine("Press d-pad right or left to change collect distance");
                    telemetry.addLine("Press 'right trigger' if the blocks are close together");
                    telemetry.addLine();
                    telemetry.addData("Collect distance", distanceToCollect);
                    telemetry.addData("Smooshed Blocks", smooshedBlocks);
                } else {
                    telemetry.addLine();
                }
                telemetry.addData("Sleep time", sleepTime);
                telemetry.addData(alliance == RED ? "Red" : "Blue", startingPosition == WAREHOUSE ? "Warehouse" : "Storage");
                telemetry.addData(finishPosition == CSP ? "collect, score, and Park" : "park", spinDucker ? "spin ducker" : "");
            }
            telemetry.update();
        }
        //*************************************** DRIVER HIT PLAY **************************************************
        if (!xIsPressed) {
            robot.drivetrain.initializeGyroAndEncoders();
        }
        if (finishPosition == PARK) {
            useVuforia = true;
        }
        robot.startTotalTime();
        scorePosition = robot.vuforia.findDuckPosition();

        sleep(sleepTime); //driver-selected sleep time

        robot.driveToHub();
        scoreFreight();

        if(startingPosition == WAREHOUSE){
            robot.collect();
            if (finishPosition == CSP) {
                robot.scoreAndPark();
            }
        }else if(startingPosition == STORAGE){
            //make sure to run down slide first
            if (spinDucker) {
                robot.goDuck();
            }
            if (finishPosition == CSP) {
                robot.collect();
                robot.scoreAndPark();
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
        if (startingPosition == WAREHOUSE){
            robot.slide.transporter.autoScore();
        } else {
            robot.slide.transporter.scoreFreight();
            runtime.reset();
            if (useVuforia) {
                robot.vuforia.switchCamera(VUFORIA);
                currentCameraMode = VUFORIA;
            } else {
                robot.vuforia.switchCamera();
            }
            if (runtime.seconds() < SCORE_TIME) {
                double timeDifference = SCORE_TIME - runtime.seconds();
                runtime.reset();
                while (runtime.seconds() < timeDifference){
                }
            }
            robot.slide.transporter.carryFreight();
            telemetry.addData("switched Camera", true);
        }
    }
}
