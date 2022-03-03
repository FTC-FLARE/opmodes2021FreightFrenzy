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
        if (!opMode.freightCollected) {
            runtime.reset();
            while (!opMode.freightCollected && runtime.seconds() < TIMEOUT_TIME) {
                setFreightCollected();
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

    public void setFreightCollected() {
        if ((freightDetector.getDistance(DistanceUnit.CM) < 37)) {
            opMode.telemetry.addData("Distance (cm)", "%.3f", freightDetector.getDistance(DistanceUnit.CM));
            opMode.sleep(300);
            opMode.freightCollected = true;
        } else {
            opMode.freightCollected = false;
        }
    }
}