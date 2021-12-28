package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Any additional import statements will go here

public class MM_Capper {
    // this gives us access to all opMode information
    private LinearOpMode opMode;

/*
    declare private class variables for all hardware and states the object “has” - Ex:
        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
*/

/*
    Declare any other class variables & constants – Ex:
        static final double WHEEL_DIAMETER_INCHES = 4.0;
*/


    // Constructor
    public MM_Capper(LinearOpMode opMode){
        this.opMode = opMode;

/*
        initialize hardware from the configuration data - Ex:
            leftDrive = opMode.hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive  = opMode.hardwareMap.get(DcMotor.class, "right_drive");
*/

/*
        do any other initialization that needs to happen - Ex:
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
*/
    }

/*
    write methods for what the object “does” – private or public - Ex:
        public void setMotorPower(double leftPower, double rightPower) {
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }
*/
}
