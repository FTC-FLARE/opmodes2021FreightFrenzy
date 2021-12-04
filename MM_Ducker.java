package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Ducker {

    // this gives us access to all opMode information
    private LinearOpMode opMode;

    private ElapsedTime runtime = new ElapsedTime();
    static final double MOTOR_POWER = 1.00;
    private DcMotor DuckerMotor= null;

    // Constructor
    public MM_Ducker(LinearOpMode opMode){
        this.opMode = opMode;
        DuckerMotor = opMode.hardwareMap.get(DcMotor.class, "Ducker");
        DuckerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void DuckerAuto(double timeoutTime) {
        runtime.reset();

        while (opMode.opModeIsActive() && runtime.seconds() < timeoutTime) {
            DuckerMotor.setPower(MOTOR_POWER);
        }

        DuckerMotor.setPower(0);
    }

    public void DuckManual() {
        if (opMode.gamepad1.right_bumper) {
             DuckerMotor.setPower(MOTOR_POWER);
        } else if (opMode.gamepad1.left_bumper) {
            DuckerMotor.setPower(-MOTOR_POWER);
        } else {
            DuckerMotor.setPower(0);
        }

    }
}
