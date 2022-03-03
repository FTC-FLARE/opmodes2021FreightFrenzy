package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot {
    private MM_OpMode opMode;

    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Slide slide;
    public MM_Ducker ducker;
    public MM_Vuforia vuforia;
    public MM_Capper capper;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();

    static final double MIN_DRIVE_SPEED = 0.12;
    static final double MAX_DRIVE_SPEED = 0.7;
    static final double FASTER_MAX_DRIVE_SPEED = 0.8;
    static final double MIN_STRAFE_POWER = 0.22;
    static final double MAX_STRAFE_POWER = 0.8;
    static final double MIN_ROTATE_POWER = 0.12;
    static final double MAX_ROTATE_POWER = 0.7;
    static final int LEFT = -1;
    static final int RIGHT = 1;
    private static final double VUFORIA_SEARCH_TIME = 2;

    public boolean vuforiaTargetFound = false; //TODO make private after testing
    private int collectorCycle = 0;
    public boolean slideRaised = false;
    public boolean scoreDuck = false;
    private boolean detectionError = false;
    private boolean collectionError = false;
    private double originalTargetAngle = 0;
    private double duckScoreDrive = 0;
    private double targetDriveAngle = 0;

    // Constructor
    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        slide = new MM_Slide(opMode);
        ducker = new MM_Ducker(opMode);/*
        capper = new MM_Capper(opMode);*/
        if (opMode.getClass() == MM_TeleOp.class) {
            drivetrain.initOdometryServos(1);
        } else {
            drivetrain.initOdometryServos(0);
            vuforia = new MM_Vuforia(opMode);
            opMode.pTurnController.setOutputRange(MIN_ROTATE_POWER, MAX_ROTATE_POWER);
            opMode.pLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
            opMode.pRightDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
            opMode.pBackDriveController.setOutputRange(MIN_STRAFE_POWER, MAX_STRAFE_POWER);
        }
    }

    public void goDuck() {
        if (opMode.finishPosition == MM_OpMode.PARK) {
            double strafeInches = 27;
            double forwardInches = -26.5;
            double targetAngle = -90;
            double secondTargetAngle = 152;
            double correctTargetAngle = 171;
            if (opMode.alliance == MM_OpMode.BLUE) {
                strafeInches = -36;
                forwardInches = -21;
                targetAngle = -targetAngle;
                secondTargetAngle = -160;
                correctTargetAngle = -correctTargetAngle;
            }

            drivetrain.pRotateDegrees(0);
            strafeAndLowerSlide(strafeInches, 2.5);
            drivetrain.pRotateDegrees(targetAngle);
            alignWithTargetDucker();
            if (vuforia.targetFound()) {
                drivetrain.pRotateDegrees(secondTargetAngle);
                drivetrain.driveForwardInches(forwardInches);
                ducker.autoSpin();;
            } else {
                drivetrain.pRotateDegrees(correctTargetAngle);
                drivetrain.driveForwardInches(-35); // havent tested for red0
                ducker.autoSpin();
            }
        } else {
            double targetAngle = 0;
            double strafeInches = 19;
            double driveInches = -23; //Always neg
            double duckerAngle = 125;
            double timeoutStrafe = 2;
            double timeoutDrive = 1.8;
            if (opMode.alliance == MM_OpMode.BLUE) {
                duckerAngle = -167;
                strafeInches = -41.5 - opMode.alliance;
                driveInches = -12;
                timeoutStrafe = 3.1;
                timeoutDrive = 1.225;
                if (opMode.scorePosition == 1) {
                    targetAngle = -6;
                } else if (opMode.scorePosition == 2) {
                    targetAngle = -3;
                }
            } else {
                if (opMode.scorePosition == 1) {
                    targetAngle = 10;
                    strafeInches = 20.6;
                    driveInches = -25;
                } else if (opMode.scorePosition == 2) {
                    targetAngle = 5;
                    strafeInches = 19.8;
                    driveInches = -24;
                }
            }

            opMode.pLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, FASTER_MAX_DRIVE_SPEED);
            opMode.pRightDriveController.setOutputRange(MIN_DRIVE_SPEED, FASTER_MAX_DRIVE_SPEED);
            drivetrain.pRotateDegrees(targetAngle);
            strafeAndLowerSlide(strafeInches, timeoutStrafe);
            drivetrain.pRotateDegrees(duckerAngle);
            drivetrain.driveForwardInchesTimeout(driveInches, timeoutDrive);
            ducker.autoSpin();
            opMode.pLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
            opMode.pRightDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
        }
    }

    public void parkFromCarousel() {
        if (opMode.finishPosition == MM_OpMode.PARK) {
            double angle = -90;
            if (opMode.alliance == MM_OpMode.BLUE) {
                angle = -angle;
            }
            drivetrain.driveForwardInches(24);
            if (vuforiaTargetFound){
                drivetrain.pRotateDegrees(angle);
                alignWithTargetPark();
            } else {
                drivetrain.pRotateDegrees(-angle);
                drivetrain.driveForwardInches(10);
            }
        } else {
            if (detectionError) {
                if (opMode.alliance == MM_OpMode.RED) {
                    drivetrain.driveForwardInches(-7.5, 0);
                } else {
                    drivetrain.driveForwardInches(-8.5, 0);
                }
            } else if (collectionError) {
                if (opMode.alliance == MM_OpMode.RED) {
                    drivetrain.fixEncoderPriorTargets();
                    drivetrain.pRotateDegrees(originalTargetAngle);
                    drivetrain.driveForwardInches(-26);
                    drivetrain.pRotateDegrees(0);
                } else {
                    drivetrain.fixEncoderPriorTargets();
                    drivetrain.pRotateDegrees(originalTargetAngle);
                    drivetrain.driveForwardInches(-25);
                    drivetrain.pRotateDegrees(0);
                    runtime.reset();
                    while (runtime.seconds() < 1.5) {
                        drivetrain.strafe(LEFT);
                    }
                    drivetrain.stop();
                }
            } else {
                opMode.pLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, FASTER_MAX_DRIVE_SPEED);
                opMode.pRightDriveController.setOutputRange(MIN_DRIVE_SPEED, FASTER_MAX_DRIVE_SPEED);
                double angleTarget = -93;
                if (opMode.alliance == MM_OpMode.BLUE) {
                    angleTarget = 100;
                }
                driveAndLowerSlide(33.5, angleTarget);
            }
        }
    }

    public void driveToHub() {
        double forwardInches = -22;
        double angleTarget = -32;
        if (opMode.startingPosition == MM_OpMode.STORAGE) {
            if(opMode.alliance == MM_OpMode.RED) {
                if (opMode.scorePosition == 2) {
                    forwardInches = -23;
                    angleTarget = -32;
                } else if (opMode.scorePosition == 1) {
                    forwardInches = -25.5;
                    angleTarget = -32;
                }
            } else {
                angleTarget = 32;
                forwardInches = -21;
                if (opMode.scorePosition == 1) {
                    forwardInches = -24.5;
                } else if (opMode.scorePosition == 2) {
                    forwardInches = -21.5;
                }
            }
        } else {
            angleTarget = 31.5;
            forwardInches = -20.5;

            if (opMode.alliance == MM_OpMode.RED) {
                if (opMode.scorePosition == 1) {
                    angleTarget = 32;
                    forwardInches = -24.75;
                }//2 is same as 3

            } else {
                angleTarget = -32.5;
                forwardInches = -22.25;
                if (opMode.scorePosition == 1) {
                    forwardInches = -26;
                    angleTarget = -33.5;
                } else if (opMode.scorePosition == 2) {
                    forwardInches = -24;
                    angleTarget = -35;
                }
            }
        }
        driveAndRaiseSlide(forwardInches, angleTarget);
        slide.runSlideToScore();
        opMode.freightCollected = false;
    }

    public void storagePark() {
        if (opMode.spinDucker) {
            parkFromCarousel();
        } else {
            //clean up later with dead encoders
            double targetHeading;
            //this variable is to straighten robot out after parking
            double secondTargetHeading;
            double forwardInches;

            forwardInches = 50;

            if (opMode.alliance == MM_OpMode.BLUE) {
                if (opMode.startingPosition == MM_OpMode.STORAGE) {
                    forwardInches = 47;
                }
                targetHeading = -80;
                if (opMode.scorePosition == 1) {
                    targetHeading = -82.75;
                } else if (opMode.scorePosition == 3) {
                    targetHeading = -80;
                }
                secondTargetHeading = -90;
            } else {
                if (opMode.startingPosition == MM_OpMode.WAREHOUSE) {
                    forwardInches = 47;
                } else {
                    forwardInches = 49;
                }

                targetHeading = 78;

                secondTargetHeading = 90;

                if (opMode.scorePosition == 1) {
                    targetHeading = 77;
                } else if (opMode.scorePosition == 3) {
                    targetHeading = 80;
                }
            }

            drivetrain.pRotateDegrees(targetHeading);
            drivetrain.driveForwardInches(forwardInches);
            drivetrain.pRotateDegrees(secondTargetHeading);
        }
    }

    public void collect() {
        if (opMode.startingPosition == MM_OpMode.WAREHOUSE) {
            double angle = 88.35;
            double secondAngle = 94;
            double straightInches = 7;
            double strafeInches = 41;

            if (opMode.scorePosition == 1) {
                straightInches = 9;
                strafeInches = 44;
            } else if (opMode.scorePosition == 2) {
                straightInches = 8;
                strafeInches = 42;
            }


            if (opMode.alliance == MM_OpMode.BLUE) {
                angle = -angle;
                secondAngle = - secondAngle;
                strafeInches = -strafeInches;
                straightInches = 5.5;
            }
            drivetrain.pRotateDegrees(angle);
            drivetrain.driveForwardInches(straightInches);
            strafeAndLowerSlide(strafeInches, 2.4);
            collector.setFreightCollected();
            collector.collect();
            drivetrain.driveForwardInches(opMode.distanceToCollect);
            double collectStopDistance = opMode.distanceToCollect + 6;
            runtime.reset();
            while (opMode.opModeIsActive() && !opMode.freightCollected && opMode.distanceToCollect < collectStopDistance) {
                if (collectorCycle == 0) {
                    collector.autoStop();
                } else if (collectorCycle == 1) {
                    collector.dispense();
                    opMode.sleep(400);
                    collector.collect();
                } else if (collectorCycle == 2) {
                    drivetrain.driveForwardInches(2);
                    opMode.distanceToCollect += 2;
                } else if (collectorCycle == 3) {
                    drivetrain.pRotateDegrees(secondAngle);
                } else if (collectorCycle == 4) {
                    drivetrain.pRotateDegrees(angle);
                    collectorCycle = -1;
                }
                collector.setFreightCollected();
                collectorCycle += 1;
            }
            handleScoreAgain();
        } else {
            if (opMode.alliance == MM_OpMode.RED) {
                double detectTargetAngle = 7.5;
                collector.collect();
                drivetrain.pRotateDegrees(detectTargetAngle);
                drivetrain.fixEncoderPriorTargets();
                drivetrain.driveForwardInches(-13.5);

                boolean seesTfod = false;
                runtime.reset();
                while (!seesTfod & runtime.seconds() < 0.9) {
                    seesTfod = vuforia.checkTfod();
                    collector.setFreightCollected();
                    if (opMode.freightCollected) {
                        seesTfod = true;
                    }
                }
                if (seesTfod) {
                    seesTfod = true;
                    double duckPixel = 0;
                    if (opMode.freightCollected) {
                        duckPixel = 225;
                    } else {
                        duckPixel = vuforia.duckLeftPixel();
                    }
                    opMode.telemetry.update();
                    double targetAngle = 10.9;
                    boolean duckPixelIsHandled = false;
                    int duckPixelThreshold = 350;
                    if (duckPixel < 475) {
                        while (!duckPixelIsHandled) {
                            if (duckPixel > duckPixelThreshold) {
                                duckPixelIsHandled = true;
                            } else if (duckPixelThreshold < 201) {
                                duckPixelThreshold -= 50;
                                targetAngle += 2;
                            } else {
                                duckPixelThreshold -= 50;
                                targetAngle += 1.05;
                            }
                        }
                        originalTargetAngle = targetAngle;
                        drivetrain.driveForwardInches(24, targetAngle);

                        runtime.reset();
                        while (!opMode.freightCollected && runtime.seconds() < 2.5) {
                            collector.setFreightCollected();
                        }
                        translateDrive(duckPixelThreshold);
                        drivetrain.pRotateDegrees(targetDriveAngle);
                        collector.setFreightCollected();
                        if (!opMode.freightCollected) {
                            opMode.sleep(300);
                            collector.setFreightCollected();
                        }
                        if (opMode.freightCollected) {
                            scoreDuck = true;
                        } else {
                            collectionError = true;
                        }
                    } else {
                        detectionError = true;
                    }
                }
                if (!seesTfod) {
                    detectionError = true;
                }
            } else {
                collector.collect();
                drivetrain.driveForwardInches(16);
                drivetrain.pRotateDegrees(-11);
                drivetrain.fixEncoderPriorTargets();

                boolean seesTfod = false;
                runtime.reset();
                while (!seesTfod & runtime.seconds() < 0.9) {
                    seesTfod = vuforia.checkTfod();
                    collector.setFreightCollected();
                    if (opMode.freightCollected) {
                        seesTfod = true;
                    }
                }
                if (seesTfod) {
                    double duckPixel = 0;
                    if (opMode.freightCollected) {
                        duckPixel = 225;
                    } else {
                        duckPixel = vuforia.duckRightPixel();
                    }
                    opMode.telemetry.update();
                    double targetAngle = 8;
                    boolean duckPixelIsHandled = false;
                    int duckPixelThreshold = 350;
                    if (duckPixel < 550) {
                        while (!duckPixelIsHandled) {
                            if (duckPixel > duckPixelThreshold) {
                                duckPixelIsHandled = true;
                            } else if (duckPixelThreshold < 201) {
                                duckPixelThreshold -= 50;
                                targetAngle += 3.5;
                            } else {
                                duckPixelThreshold -= 50;
                                targetAngle += 1.3;
                            }
                        }
                        targetAngle = -targetAngle;
                        originalTargetAngle = targetAngle;
                        drivetrain.driveForwardInches(22, targetAngle);

                        runtime.reset();
                        while (!opMode.freightCollected && runtime.seconds() < 2.5) {
                            collector.setFreightCollected();
                        }
                        translateDrive(duckPixelThreshold);
                        drivetrain.pRotateDegrees(targetDriveAngle);
                        collector.setFreightCollected();
                        if (!opMode.freightCollected) {
                            opMode.sleep(300);
                            collector.setFreightCollected();
                        }
                        if (opMode.freightCollected) {
                            scoreDuck = true;
                        } else {
                            collectionError = true;
                        }
                    } else {
                        detectionError = true;
                    }
                }
                if (!seesTfod) {
                    detectionError = true;
                }
            }
        }
    }

    public void scoreAndPark() {
        if (opMode.startingPosition == MM_OpMode.WAREHOUSE) {
            slide.fixSensor();
            if (opMode.scoreAgain) {
                double secondAngle = 88.35;
                double firstStrafeInches = -12;
                double secondStrafeInches = 32;
                double firstAngle = 29.5;
                double firstForwardInches = -12;
                double secondForwardInches = 46;
                //better way to do this
                if (opMode.alliance == MM_OpMode.RED) {
                    firstForwardInches = -15.25;
                    if (opMode.scorePosition == 1) {
                        firstAngle = 26;
                        firstForwardInches = -14.25;
                    } else if (opMode.scorePosition == 2) {
                        firstAngle = 29;
                        firstForwardInches = -13;
                    }
                } else {
                    secondAngle = -secondAngle;
                    firstStrafeInches = -firstStrafeInches;
                    secondStrafeInches = -secondStrafeInches;
                    firstAngle = -23;
                    secondForwardInches = 39;
                    if (opMode.scorePosition == 1) {
                        firstAngle = -25;
                    } else if (opMode.scorePosition == 2) {
                        firstAngle = -22;
                    }
                }

                opMode.scorePosition = 3;
                drivetrain.driveForwardInches(-opMode.distanceToCollect - 1);
                drivetrain.strafeInches(firstStrafeInches);
                driveAndRaiseSlide(firstForwardInches, firstAngle);
                slide.runSlideToScore();
                slide.transporter.autoScore();
                drivetrain.pRotateDegrees(secondAngle);
                strafeAndLowerSlide(secondStrafeInches, 2.3);
                collector.setFreightCollected();
                slide.fixSensor();
                collector.collect();
                drivetrain.driveForwardInches(secondForwardInches);
                collector.autoStop();
            } else {
                collector.stop();
            }
        } else {
            if (totalTime.seconds() > 24) {
                collectionError = true;
                scoreDuck = false;
            } else if (scoreDuck) {
               opMode.scorePosition = 2;
               driveAndRaiseSlide(duckScoreDrive, targetDriveAngle);
               slide.runSlideToScore();
               collector.stop();
               slide.transporter.autoScore();
               drivetrain.fixEncoderPriorTargets();
           }
        }
    }


    public void strafeAndLowerSlide(double inches, double timeoutTime) {
        boolean slideDone = false;
        boolean strafeDone = false;
        slide.startLowering();
        slideRaised = false;
        drivetrain.prepareToStrafe(inches);

        runtime.reset();
        while (opMode.opModeIsActive() && (!slideDone || !strafeDone)) {
            if (runtime.seconds() < 3.1 && !slideDone) {
                slideDone = slide.reachedPositionDown();
            } else {
                slideDone = true;
                slide.fixPosition();
            }

            if (!strafeDone) {
                if (runtime.seconds() < timeoutTime) {
                    drivetrain.setStrafePower();
                } else {
                    strafeDone = true;
                    drivetrain.stop();
                }
            }
            opMode.sleep(15);
        }
        slide.fixSensor();
    }

    public void driveAndLowerSlide(double inches, double angleTarget) {
        boolean slideDone = false;
        boolean driveDone = false;
        slide.startLowering();
        slideRaised = false;
        drivetrain.prepareToDrive(inches, angleTarget);

        runtime.reset();
        while (opMode.opModeIsActive() && (!slideDone || !driveDone)) {
            if (runtime.seconds() < 3.1 && !slideDone) {
                slideDone = slide.reachedPositionDown();
            } else {
                slideDone = true;
                slide.fixPosition();
            }

            if (!driveDone) {
                if (runtime.seconds() < 2.8 && !driveDone) {
                    drivetrain.reachedTargetDrive();
                } else {
                    driveDone = true;
                    drivetrain.stop();
                }
            } else {
                driveDone = true;
            }
            opMode.sleep(15);
        }
        slide.fixSensor();
    }

    public void driveAndRaiseSlide(double inches, double angleTarget) {
        boolean slideDone = false;
        boolean driveDone = false;
        slide.startRaising();
        drivetrain.prepareToDrive(inches, angleTarget);

        runtime.reset();
        if (opMode.scorePosition == 2 || opMode.scorePosition == 3) {
            while (opMode.opModeIsActive() && (!slideDone || !driveDone)) {
                if (runtime.seconds() < 4 && !slideDone) {
                    slideDone = slide.reachedPositionUp2();
                } else {
                    slideDone = true;
                    slide.stop();
                    slideRaised = true;
                }
                if (runtime.seconds() < 2.8 && !driveDone) {
                    driveDone = drivetrain.reachedTargetDrive();
                    if (driveDone) {
                        drivetrain.stop();
                        driveDone = true;
                    }
                } else {
                    driveDone = true;
                }
            }
        } else {
            while (opMode.opModeIsActive() && (!slideDone || !driveDone)) {
                if (runtime.seconds() < 2 && !slideDone) {
                    slideDone = slide.reachedPositionUp();
                } else {
                    slideDone = true;
                    slide.stop();
                    slideRaised = true;
                }
                if (runtime.seconds() < 2.8 && !driveDone) {
                    driveDone = drivetrain.reachedTargetDrive();
                    if (driveDone) {
                        drivetrain.stop();
                        driveDone = true;
                    }
                } else {
                    driveDone = true;
                }
            }
        }
    }

    private void handleScoreAgain() {
        if (runtime.seconds() < 5) {
            opMode.scoreAgain = true;
        } else {
            opMode.scoreAgain = false;
        }
    }

    private void translateDrive(int duckThreshold) {
        double xSideLength = (37.8 - ((350 - duckThreshold)/50 * 0.52));
        double constantAngle = 42.5;
        if (opMode.alliance == MM_OpMode.BLUE) {
            constantAngle = 50;
            xSideLength = (32.5 - ((350- duckThreshold)/50 * 0.52));
        }
        double ySideLength = 1.5; //was 3.3

        duckScoreDrive = -Math.hypot(xSideLength, ySideLength);
        double driveAngle = Math.atan2(ySideLength, xSideLength);
        targetDriveAngle = -(constantAngle + driveAngle); //pos for blue??
        if (opMode.alliance == MM_OpMode.BLUE) {
            targetDriveAngle = -targetDriveAngle;
        }
    }

    public void alignWithTargetDucker() {
        int direction = RIGHT;
        if (opMode.alliance == MM_OpMode.BLUE) {
            direction = LEFT;
        }
        runtime.reset();
        while (opMode.opModeIsActive() && !vuforiaTargetFound && runtime.seconds() < 3) {
            drivetrain.strafe(direction);
            vuforiaTargetFound = vuforia.targetFound();
        }
        if (vuforiaTargetFound){
            double xTarget = 0;
            double xThreshold = 0.75;
            double yTarget = 10;
            if (opMode.alliance == MM_OpMode.RED) {
                xTarget = 4;
                xThreshold = 1;
                yTarget = 16;
            }
            boolean strafeDone = false;
            boolean driveDone = false;
            while (opMode.opModeIsActive() && (!strafeDone || !driveDone)) {
                drivetrain.stop();
                drivetrain.fixEncoderPriorTargets();
                while (opMode.opModeIsActive() && (Math.abs(vuforia.getX() - xTarget) > xThreshold) && vuforia.targetFound()) {
                    if (vuforia.getX() > xTarget) {
                        direction = RIGHT;
                    } else {
                        direction = LEFT;
                    }
                    drivetrain.strafe(direction);
                }
                drivetrain.stop();
                while (opMode.opModeIsActive() && (Math.abs(vuforia.getY() - yTarget)) > 1 && vuforia.targetFound()) {
                    if (vuforia.getY() > yTarget) {
                        direction = RIGHT;
                    } else {
                        direction = LEFT;
                    }
                    drivetrain.drive(direction);
                }
                drivetrain.stop();
                if (Math.abs(vuforia.getX() - xTarget) < xThreshold) {
                    strafeDone = true;
                }
                if (Math.abs(vuforia.getY() - yTarget) < 1) {
                    driveDone = true;
                }
            }
            drivetrain.fixEncoderPriorTargets();
        }
    }

    public void alignWithTargetPark() {
        double angleTarget = 90;
        int direction = LEFT;
        if (opMode.alliance == MM_OpMode.BLUE) {
            direction = RIGHT;
            angleTarget = -angleTarget;
        }
        if (opMode.scorePosition == 1) {
            direction = -direction;
        }
        vuforiaTargetFound = false;
        while (!vuforiaTargetFound) {
            drivetrain.strafe(direction);
            vuforiaTargetFound = vuforia.targetFound();
        }
        while (opMode.opModeIsActive() && (Math.abs(vuforia.getX()) - 5) > 0.75 && vuforia.targetFound()) {
            if ((vuforia.getX() - 5) > 0) {
                direction = RIGHT;
            } else {
                direction = LEFT;
            }
            drivetrain.strafe(direction);
        }
        drivetrain.stop();
        drivetrain.pRotateDegrees(angleTarget);
        drivetrain.driveForwardInches(-10);
    }

    public void startTotalTime() {
        totalTime.reset();
    }
}