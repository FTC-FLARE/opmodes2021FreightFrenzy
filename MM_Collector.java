package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MM_Collector {
    private LinearOpMode opMode;

    private DcMotor collector = null;

    public MM_Collector(LinearOpMode opMode) {
        this.opMode = opMode;

        collector = opMode.hardwareMap.get(DcMotor.class, "collector");
    }

    public void runCollector() {
        if (opMode.gamepad2.left_bumper) {
            collector.setPower(1);
        } else if (opMode.gamepad2.right_bumper) {
            collector.setPower(-1);
        } else {
            collector.setPower(0);
        }
    }
}