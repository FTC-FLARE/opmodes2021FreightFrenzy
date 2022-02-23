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

    static final double MIN_DRIVE_SPEED = 0.12;
    static final double MAX_DRIVE_SPEED = 0.7;
    static final double MIN_STRAFE_POWER = 0.22;
    static final double MAX_STRAFE_POWER = 0.8;
    static final double MIN_ROTATE_POWER = 0.12;
    static final double MAX_ROTATE_POWER = 0.7;
    static final int LEFT = -1;
    static final int RIGHT = 1;
    private static final double VUFORIA_SEARCH_TIME = 2;

     public boolean vuforiaTargetFound = false; //TODO make private after testing

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
        double strafeInches = 36;
        double targetAngle = -90;
        double secondTargetAngle = 160;
        double correctTargetAngle = 171;
        if (opMode.alliance == MM_OpMode.RED) {

        } else {
            strafeInches = -strafeInches;
            targetAngle = -targetAngle;
            secondTargetAngle = -secondTargetAngle;
            correctTargetAngle = -correctTargetAngle;
        }

        drivetrain.pRotateDegrees(0);
        strafeAndLowerSlide(strafeInches, 2.5);
        drivetrain.pRotateDegrees(targetAngle);
        alignWithTargetDucker();
        if (vuforia.targetFound()) {
            drivetrain.pRotateDegrees(secondTargetAngle);
            drivetrain.driveForwardInches(-21);
            ducker.autoSpin();;
        } else {
            drivetrain.pRotateDegrees(correctTargetAngle);
            drivetrain.driveForwardInches(-35);
            ducker.autoSpin();
        }
    }

    public void parkFromCarousel() {
        double angle = -90;
        if (opMode.alliance == MM_OpMode.BLUE) {
            angle = -angle;
        }
        drivetrain.driveForwardInches(24);
        drivetrain.pRotateDegrees(angle);
        if (vuforiaTargetFound){
            alignWithTargetPark();
        } else {
            drivetrain.driveForwardInches(10);
        }
    }

    public void driveToHub() {
        double forwardInches = -6;
        if (opMode.startingPosition == MM_OpMode.STORAGE) {
            double angleTarget = -34;
            if(opMode.alliance == MM_OpMode.RED) {
            } else {
                angleTarget = 34;
                forwardInches = -21;
                if (opMode.scorePosition == 1) {
                    forwardInches = -25.4;
                } else if (opMode.scorePosition == 2) {
                    forwardInches = -22.5;
                }
            }
            drivetrain.driveForwardInches(forwardInches, angleTarget);
        } else {
            double angleTarget = 31.5;
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

            drivetrain.driveForwardInches(forwardInches, angleTarget);
        }
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

    public void warehouseCollect() {
        double angle = 88.35;
        double secondAngle = 94;
        double straightInches = 7;
        double strafeInches = 42;

        if (opMode.alliance == MM_OpMode.BLUE) {
            angle = -angle;
            secondAngle = - secondAngle;
            strafeInches = -strafeInches;
            straightInches = 5.5;
        }
        if (opMode.scorePosition == 1) {
            straightInches = 9;
        } else if (opMode.scorePosition == 2) {
            straightInches = 8;
        }

        drivetrain.pRotateDegrees(angle);
        drivetrain.driveForwardInches(straightInches);
        strafeAndLowerSlide(strafeInches, 2.4);
        collector.setFreightCollected();
        collector.collect();
        drivetrain.driveForwardInches(opMode.distanceToCollect);
        runtime.reset();
        while (opMode.opModeIsActive() && !opMode.freightCollected && opMode.distanceToCollect < 46) {
            collector.autoStop();
            if (!opMode.freightCollected) {
                collector.dispense();
                collector.setFreightCollected();
                opMode.sleep(400);
                collector.collect();
                collector.setFreightCollected();
                drivetrain.driveForwardInches(2);
                collector.setFreightCollected();
                opMode.distanceToCollect += 2;
                drivetrain.pRotateDegrees(secondAngle);
                collector.setFreightCollected();
                drivetrain.pRotateDegrees(angle);
            }
        }
        handleScoreAgain();
    }

    public void ScoreAndPark() {
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
                firstAngle = -27;
                secondForwardInches = 39;
                if (opMode.scorePosition == 1) {
                    firstAngle = -27;
                } else if (opMode.scorePosition == 2) {
                    firstAngle = -28;
                }
            }

            opMode.scorePosition = 3;
            drivetrain.driveForwardInches(-opMode.distanceToCollect - 1);
            drivetrain.strafeInches(firstStrafeInches);
            drivetrain.driveForwardInches(firstForwardInches, firstAngle);
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
    }


    public void strafeAndLowerSlide(double inches, double timeoutTime) {
        boolean slideDone = false;
        boolean strafeDone = false;
        slide.startLowering();
        drivetrain.prepareToStrafe(inches);

        runtime.reset();
        while (opMode.opModeIsActive() && (!slideDone || !strafeDone)) {
            if (runtime.seconds() < 3.2 && !slideDone) {
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

    public void driveAndRaiseSlide(double inches, double angleTarget) {
        boolean slideDone = false;
        boolean driveDone = false;
        slide.startRaising();
        drivetrain.prepareToDrive(inches, angleTarget);

        runtime.reset();
        while (opMode.opModeIsActive() && (!slideDone || !driveDone)) {
            if (runtime.seconds() < 2.5 && !slideDone) {
                slideDone = slide.reachedPositionUp();
            } else {
                slideDone = true;
                slide.stop();
            }
            if (!driveDone) {
                driveDone = drivetrain.reachedTargetDrive();
                if (driveDone) {
                    drivetrain.stop();
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
            boolean strafeDone = false;
            boolean driveDone = false;
            while (opMode.opModeIsActive() && (!strafeDone || !driveDone)) {
                drivetrain.stop();
                drivetrain.fixEncoderPriorTargets();
                while (opMode.opModeIsActive() && (Math.abs(vuforia.getX()) > 0.75) && vuforia.targetFound()) {
                    if (vuforia.getX() > 0) {
                        direction = RIGHT;
                    } else {
                        direction = LEFT;
                    }
                    drivetrain.strafe(direction);
                }
                drivetrain.stop();
                while (opMode.opModeIsActive() && (Math.abs(vuforia.getY() - 10)) > 1 && vuforia.targetFound()) {
                    if (vuforia.getY() - 10 > 0) {
                        direction = RIGHT;
                    } else {
                        direction = LEFT;
                    }
                    drivetrain.drive(direction);
                }
                drivetrain.stop();
                if (vuforia.getX() < 0.75 && vuforia.getX() > -0.75) {
                    strafeDone = true;
                }
                if (vuforia.getY() < 11 && vuforia.getY() > 9) {
                    driveDone = true;
                }
            }
            drivetrain.fixEncoderPriorTargets();
        }
    }

    public void alignWithTargetPark() {
        int direction = LEFT;
        if (opMode.alliance == MM_OpMode.BLUE) {
            direction = RIGHT;
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
        drivetrain.driveForwardInches(8);
    }
}