package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MM_Collector {
    private LinearOpMode opMode;

    private DcMotor collector = null;

    private ElapsedTime runtime = new ElapsedTime();

    private final double TIMEOUT_TIME = 2;

    public MM_Collector(LinearOpMode opMode) {
        this.opMode = opMode;
        collector = opMode.hardwareMap.get(DcMotor.class, "collector");
    }

    public void runCollector() {
        if (opMode.gamepad2.left_bumper) {
            collect();
        } else if (opMode.gamepad2.right_bumper) {
            dispense();
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

    private void collect() {
        collector.setPower(1);
    }

    private void dispense() {
        collector.setPower(-1);
    }

    private void stop() {
        collector.setPower(0);
    }
}