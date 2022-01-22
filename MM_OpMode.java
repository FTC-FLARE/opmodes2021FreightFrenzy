package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public abstract class MM_OpMode extends LinearOpMode {

    static double DRIVE_P_COEFFICIENT = 0.000095;
    static final int RED = 1;
    static final int BLUE = 2;
    static final int WAREHOUSE = 1;
    static final int STORAGE = 2;
    static final int OOTW = 1;
    static final int PARK = 2;
    public long sleepTime = 0;
    public int alliance = RED;
    public int startingPosition = WAREHOUSE;
    public int finishPosition = OOTW;
    public boolean spinDucker = false;

    public MM_Robot robot = new MM_Robot(this);
    public MM_P_Controller pTurnController = new MM_P_Controller(this, 2, .01);
    public MM_P_Controller pLeftDriveController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT);
    public MM_P_Controller pRightDriveController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT);
    public MM_P_Controller pBackDriveController = new MM_P_Controller(this,(5/3), DRIVE_P_COEFFICIENT); //TODO Make P larger
}
