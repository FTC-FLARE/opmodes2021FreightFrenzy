package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

public class MM_Robot {
    private MM_OpMode opMode;

    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Slide slide;
    public MM_Ducker ducker;
    public MM_Vuforia vuforia;

    static final double MIN_DRIVE_SPEED = 0.12;
    static final double MAX_DRIVE_SPEED = 0.6;
    static final double MIN_STRAFE_POWER = 0.22;
    static final double MAX_STRAFE_POWER = .8;
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

    public void goDuck(double duckPosition) {
        double forwardInches = 42;
        double targetAngle = 103;
        double secondTargetAngle = -37;
        double timeoutTime = 0.90; //default for red 3
        if (duckPosition == 1 && opMode.alliance == MM_OpMode.RED) {
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

        if (duckPosition == 2) {
            timeoutTime = 0.9;
        } else if (duckPosition == 1) {
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

    public void scoreOnHub(int duckPosition) {
        double forwardInches = -6;
        double strafeInches = 0;
        if (opMode.startingPosition == MM_OpMode.STORAGE) {
            if(opMode.alliance == MM_OpMode.RED) {

                strafeInches = 16;

                //determine driving position
                if(duckPosition == 1) {
                    forwardInches = -8.75;
                } else if(duckPosition == 2) {
                    forwardInches = -6;
                }
            } else {

                forwardInches = -6;
                strafeInches = -27;

                //determine driving position (MEASURE)
                if (duckPosition == 1) {
                    forwardInches = -6.75;
                } else if (duckPosition == 2) {
                    forwardInches = -3.5;
                }
            }
            drivetrain.driveForwardInches(12);
            drivetrain.strafeInches(strafeInches);
            drivetrain.pRotateDegrees(179);
            drivetrain.driveForwardInches(forwardInches);
        } else {
            double angleTarget = -35;
            forwardInches = -26;

            if (duckPosition == 1) {
                forwardInches = -22;
            } else if (duckPosition == 2) {
                forwardInches = -24;
            }

            if (opMode.alliance == MM_OpMode.RED) {
                angleTarget = 35;
            }

            drivetrain.driveForwardInches(forwardInches, angleTarget);
        }
        slide.goToPositionAuto(duckPosition);
        slide.autoCollectPosition(duckPosition);
    }

    public void storagePark(double duckPosition) {
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
                if (duckPosition == 1) {
                    targetHeading = -82.75;
                } else if (duckPosition == 3) {
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

                if (duckPosition == 1) {
                    targetHeading = 77;
                } else if (duckPosition == 3) {
                    targetHeading = 80;
                }
            }

            drivetrain.pRotateDegrees(targetHeading);
            drivetrain.driveForwardInches(forwardInches);
            drivetrain.pRotateDegrees(secondTargetHeading);
        }
    }

    public void warehousePark() {
        double angle = 90;
        double strafeInches = 40;

        if (opMode.alliance == MM_OpMode.BLUE) {
            angle = -angle;
            strafeInches = -strafeInches;
        }

        drivetrain.pRotateDegrees(angle);
        drivetrain.strafeInches(strafeInches);
        drivetrain.driveForwardInches(48);
    }
}