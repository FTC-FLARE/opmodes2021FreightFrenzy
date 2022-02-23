package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {
    private MM_OpMode opMode;

    private DcMotor collector = null;
    private DistanceSensor freightDetector = null;

    private ElapsedTime runtime = new ElapsedTime();
    private boolean isHandled = false;
    private double collectPower = 1;

    private final double TIMEOUT_TIME = 2.25;

    public MM_Collector(MM_OpMode opMode) {
        this.opMode = opMode;
        collector = opMode.hardwareMap.get(DcMotor.class, "collector");
        freightDetector = opMode.hardwareMap.get(DistanceSensor.class, "freightDetector");
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

    public void autoStop() {
        if (!opMode.freightCollected && !collectedFreight()) {
            runtime.reset();
            boolean freightCollected = false;
            while (!freightCollected && runtime.seconds() < TIMEOUT_TIME) {
                freightCollected = collectedFreight();
            }
            setFreightCollected();
        }
        if (opMode.freightCollected) {
            stop();
        }
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

    public boolean collectedFreight() {
        opMode.telemetry.addData("Distance (cm)", "%.3f", freightDetector.getDistance(DistanceUnit.CM));
        if (freightDetector.getDistance(DistanceUnit.CM) > 1) {
            opMode.sleep(300);
            return true;
        } else {
            return false;
        }
    }

    public void setFreightCollected() {
        if (collectedFreight()) {
            opMode.freightCollected = true;
        } else {
            opMode.freightCollected = false;
        }
    }
}