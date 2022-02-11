package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Collector {
    private MM_OpMode opMode;

    private DcMotor collector = null;

    private ElapsedTime runtime = new ElapsedTime();

    private final double TIMEOUT_TIME = 1.5;

    public MM_Collector(MM_OpMode opMode) {
        this.opMode = opMode;
        collector = opMode.hardwareMap.get(DcMotor.class, "collector");
    }

    public void runCollector() {
        if (opMode.gamepad2.left_bumper) {
            dispense();
        } else if (opMode.gamepad2.right_bumper) {
            collect();
        } else {
            stop();
        }
    }

    public void autoCollect() {
        collect();
        runtime.reset();
        while (opMode.opModeIsActive() && runtime.seconds() < TIMEOUT_TIME) {
        }
        stop();
    }

    public void collect() {
        collector.setPower(-1);
    }

    public void dispense() {
        collector.setPower(1);
    }

    public void stop() {
        collector.setPower(0);
    }
}