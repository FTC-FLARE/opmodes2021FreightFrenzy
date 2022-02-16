package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class MM_OpMode extends LinearOpMode {

    static double DRIVE_P_COEFFICIENT = 0.000079;
    static double STRAFE_P_COEFFICIENT = 0.000155;
    static final int RED = 1;
    static final int BLUE = 2;
    static final int WAREHOUSE = 1;
    static final int STORAGE = 2;
    static final int CSP = 1;
    static final int PARK = 2;
    public long sleepTime = 0;
    public int alliance = RED;
    public int startingPosition = WAREHOUSE;
    public int finishPosition = CSP;
    public boolean spinDucker = false;
    public int scorePosition = 3;
    public int distanceToCollect = 39;

    public MM_Robot robot = new MM_Robot(this);
    public MM_P_Controller pTurnController = new MM_P_Controller(this, 1, .015);
    public MM_P_Controller pLeftDriveController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT);
    public MM_P_Controller pRightDriveController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT);
    public MM_P_Controller pBackDriveController = new MM_P_Controller(this,(5/3), STRAFE_P_COEFFICIENT); //TODO Make P larger
}
