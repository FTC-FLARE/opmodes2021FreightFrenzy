package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
// Any additional import statements will go here

public class MM_Ducker {

/*
    declare private class variables for all hardware and states the object “has” - Ex:
        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
*/

    // this gives us access to all opMode information
    private LinearOpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double MOTOR_POWER = 0.69;
    private DcMotor DuckerMotor= null;

    // Constructor
    public MM_Ducker(LinearOpMode opMode){
        this.opMode = opMode;
        DuckerMotor = opMode.hardwareMap.get(DcMotor.class, "Ducker");

    }

    public void DuckerAuto(double timeoutTime) {

        runtime.reset();
        while (runtime.seconds() < timeoutTime) {
            DuckerMotor.setPower(MOTOR_POWER);
        }
        DuckerMotor.setPower(0);

    }

    public void DuckManual() {

        //while right bumper is pressed, move ducker motor
        while (opMode.gamepad1.right_bumper) {
             DuckerMotor.setPower(MOTOR_POWER);
        }

        DuckerMotor.setPower(0);

    }



}
