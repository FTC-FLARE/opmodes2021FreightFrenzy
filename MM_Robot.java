package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Robot {
    private MM_OpMode opMode;

    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Slide slide;
    public MM_Ducker ducker;
    public MM_Vuforia vuforia;

    private ElapsedTime runtime = new ElapsedTime();

    static final double MIN_DRIVE_SPEED = 0.12;
    static final double MAX_DRIVE_SPEED = 0.7;
    static final double MIN_STRAFE_POWER = 0.22;
    static final double MAX_STRAFE_POWER = 0.8;
    static final double MIN_ROTATE_POWER = 0.12;
    static final double MAX_ROTATE_POWER = 0.7;

    // Constructor
    public MM_Robot(MM_OpMode opMode){
        this.opMode = opMode;
    }

    public void init() {
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        slide = new MM_Slide(opMode);
        ducker = new MM_Ducker(opMode);
        if (opMode.getClass() == MM_TeleOp.class) {
            drivetrain.initOdometryServos(1);
        } else {
            drivetrain.initOdometryServos(0);
            vuforia = new MM_Vuforia(opMode);
            drivetrain.initializeGyro();
            opMode.pTurnController.setOutputRange(MIN_ROTATE_POWER, MAX_ROTATE_POWER);
            opMode.pLeftDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
            opMode.pRightDriveController.setOutputRange(MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
            opMode.pBackDriveController.setOutputRange(MIN_STRAFE_POWER, MAX_STRAFE_POWER);
        }
    }

    public void goDuck() {
        double forwardInches = 42;
        double targetAngle = 103;
        double secondTargetAngle = -37;
        double timeoutTime = 0.90; //default for red 3
        if (opMode.scorePosition == 1 && opMode.alliance == MM_OpMode.RED) {
            secondTargetAngle = -27;
        }
        if (opMode.alliance == MM_OpMode.BLUE) {
            forwardInches = 44;
            targetAngle = -100;
            secondTargetAngle = 20;
            timeoutTime = 0.75; //default for blue 3
        }

        drivetrain.pRotateDegrees(targetAngle);
        drivetrain.driveForwardInches(forwardInches);
        drivetrain.pRotateDegrees(secondTargetAngle);

        if (opMode.scorePosition == 2) {
            timeoutTime = 0.9;
        } else if (opMode.scorePosition == 1) {
            timeoutTime = 1.1;
        }

/*        runtime.reset(); //TODO either use encoders or drive using time method
        drivetrain.startMotors(-0.2, -0.2, -0.2,-0.2);
        while (opMode.opModeIsActive() && runtime.seconds() < timeoutTime) {
        }
        stop();*/
        ducker.autoSpin();
    }

    public void parkFromCarousel() {
        drivetrain.driveForwardInches(6);
    }

    public void scoreOnHub() {
        double forwardInches = -6;
        double strafeInches = 0;
        if (opMode.startingPosition == MM_OpMode.STORAGE) {
            if(opMode.alliance == MM_OpMode.RED) {

                strafeInches = 16;

                //determine driving position
                if(opMode.scorePosition == 1) {
                    forwardInches = -8.75;
                } else if(opMode.scorePosition == 2) {
                    forwardInches = -6;
                }
            } else {

                forwardInches = -6;
                strafeInches = -27;

                //determine driving position (MEASURE)
                if (opMode.scorePosition == 1) {
                    forwardInches = -6.75;
                } else if (opMode.scorePosition == 2) {
                    forwardInches = -3.5;
                }
            }
            drivetrain.driveForwardInches(12);
            drivetrain.strafeInches(strafeInches);
            drivetrain.pRotateDegrees(179);
            drivetrain.driveForwardInches(forwardInches);
        } else {
            double angleTarget = 31.5;
            forwardInches = -20.5;

            if (opMode.alliance == MM_OpMode.RED) {
                if (opMode.scorePosition == 1) {
                    angleTarget = 32;
                    forwardInches = -25;
                }//2 is same as 3

            } else {
                angleTarget = -32.5;
                forwardInches = -22.25;
                if (opMode.scorePosition == 1) {
                    forwardInches = -26;
                    angleTarget = -33.5;
                } else if (opMode.scorePosition == 2) {
                    forwardInches = -24;
                    angleTarget = -33.75;
                }
            }

            drivetrain.driveForwardInches(forwardInches, angleTarget);
        }
        slide.runSlideAndScoreFreight();
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
        double straightInches = 7;
        double strafeInches = -40;

        if (opMode.alliance == MM_OpMode.BLUE) {
            angle = -angle;
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
        collector.collect();
        drivetrain.driveForwardInches(opMode.distanceToCollect);
        collector.autoStop();
    }

    public void ScoreAndPark() {
        double secondAngle = 88.35;
        double firstStrafeInches = 12;
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
            firstAngle = -firstAngle;
            secondForwardInches = 39;
            if (opMode.scorePosition == 1) {
                firstAngle = -26;
            } else if (opMode.scorePosition == 2) {
                firstAngle = -29;
            }
        }

        opMode.scorePosition = 3;
        drivetrain.driveForwardInches(-opMode.distanceToCollect - 1);
        drivetrain.strafeInches(firstStrafeInches);
        drivetrain.driveForwardInches(firstForwardInches, firstAngle);
        slide.runSlideAndScoreFreight();
        drivetrain.pRotateDegrees(secondAngle);
        strafeAndLowerSlide(secondStrafeInches, 2.3);
        collector.collect();
        drivetrain.driveForwardInches(secondForwardInches);
        collector.autoStop();
    }


    public void strafeAndLowerSlide(double inches, double timeoutTime) {
        boolean slideDone = false;
        boolean strafeDone = false;
        slide.startLowering();
        drivetrain.prepareToStrafe(inches);

        runtime.reset();
        while (opMode.opModeIsActive() && (!slideDone || !strafeDone)) {
            if (!slideDone) {
                slideDone = slide.reachedPosition();
            }
            if (!strafeDone) {
                if (runtime.seconds() < timeoutTime) {
                    drivetrain.setStrafePower();
                } else {
                    strafeDone = true;
                    drivetrain.stop();
                }
            }
        }
    }
}