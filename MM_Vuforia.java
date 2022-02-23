package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class MM_Vuforia {
    private MM_OpMode opMode;  // Give us access to the calling OpMode

    //define constants for field navigation
    private static final String VUFORIA_KEY = "AZ5woGn/////AAABmSDumo9pA0BDovmvaV5gG7wLT6ES1QrKcI14JsHiEtQ7Gb6e+KM8ILBQGt8hjfHFNwKixlUDQ6vuz0AdKiYelGz5KcfJ9UV4xCMuDxDGvzOqYIS46QLHeFtsx4c4EP5o5a+H4ZM4crit1cva6avYORJXAH4EYCNluvawI+qm7qOru223kxOmNw83qfl17h9ASLtxxZuZ6OiAnQEq0OsSJf5n43QzVRFI55ZYdVAq+7bSeBEMptf1ZbrzvAZWnq8diTq+ojaADlkeZloub6tSLn4OqqbVtnjk65dNVejK2nTY1y7j7v0BQAkqc0w6oMkg30ynxOoyGid1xjSDDEaS1DvbVjQO0ODZZ4O9v6C30dtQ";
    private static final float MM_PER_INCH = 25.4f;
    private static final float MM_TARGET_HEIGHT = 6 * MM_PER_INCH;
    private static final float HALF_FIELD = 72 * MM_PER_INCH;
    private static final float HALF_TILE = 12 * MM_PER_INCH;
    private static final float ONE_AND_HALF_TILE = 36 * MM_PER_INCH;

    //declare webcam & instance variables
    private WebcamName frontCam, backCam = null;
    private SwitchableCamera switchableCamera = null;
    private ElapsedTime runtime = new ElapsedTime();

    private OpenGLMatrix targetLocation = null;
    private VuforiaLocalizer vLocalizer = null;
    private VuforiaTrackables targets = null;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

    private boolean targetVisible = false;

    public MM_Vuforia(MM_OpMode opMode) {
        this.opMode = opMode;
        vuforiaInit();
        tfodInit();
    }

    public int findDuckPosition() {
        //possibly change to void statement to just drive
        double duckLeftPixel = -1;
        int duckPosition = 0;
        runtime.reset();

        while ((runtime.seconds() < 0.5) && duckLeftPixel < 0) {
            if (tfod != null) {
                List<Recognition> tfodRecognitions = tfod.getRecognitions();
                if (!tfodRecognitions.isEmpty()) {
                    Recognition recognition = tfodRecognitions.get(0);
                    duckLeftPixel = Math.abs(recognition.getLeft());
                    opMode.telemetry.addData("Duck Found left:", "%.03f", duckLeftPixel);
                    opMode.telemetry.update();
                }
            }
        }

        if (duckLeftPixel < 0) {
            duckPosition = 3;
        } else if (duckLeftPixel > 220) {
            duckPosition = 2;
        } else {
            duckPosition = 1;
        }
        return duckPosition;
    }

    public void switchCameraMode(int mode) {
        if (mode == MM_OpMode.VUFORIA) {
            deactivateTfod();
            targets.activate();
            switchableCamera.setActiveCamera(frontCam);
        } else {
            deactivateTargets();
            tfod.activate();
            switchableCamera.setActiveCamera(backCam);
        }
    }

    public void deactivateTargets() {
        targets.deactivate();
    }

    public void deactivateTfod() {
        tfod.deactivate();
    }

    public boolean targetFound() {

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                targetLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();
                opMode.telemetry.addData("X Distance", "%5.1f inches", getX());
                opMode.telemetry.addData("Y Distance", "%5.1f inches", getY());
                opMode.telemetry.addData("Bearing to Target", "%3.0f Degrees", getHeading());
                opMode.telemetry.addData("Image Found", trackable.getName());
                if (targetLocation != null) {
                    return true;
                }
            }
        }
        return false;
    }

    public double getX() {
        return targetLocation.getTranslation().get(0) / MM_PER_INCH;
    }

    public double getY() {
        return targetLocation.getTranslation().get(2) / MM_PER_INCH;
    }//

    public double getHeading() {
        return Math.toDegrees(Math.asin(getX()/ Math.hypot(getX(), getY())));
    }

    private void vuforiaInit() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        frontCam = opMode.hardwareMap.get(WebcamName.class, "FrontCam");
        backCam = opMode.hardwareMap.get(WebcamName.class, "BackCam");

//         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = backCam;  // We indicate which camera we wish to use.
        parameters.useExtendedTracking = false;  // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(frontCam, backCam);

        vLocalizer = ClassFactory.getInstance().createVuforia(parameters);  //  Instantiate the Vuforia engine
        switchableCamera = (SwitchableCamera) vLocalizer.getCamera();
        switchableCamera.setActiveCamera(backCam);

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        targets = vLocalizer.loadTrackablesFromAsset("FreightFrenzy");
        allTrackables.addAll(targets);

        // Name and locate each trackable object
        allTrackables.get(0).setName("Blue Storage");
        allTrackables.get(1).setName("Blue Alliance Wall");
        allTrackables.get(2).setName("Red Storage");
        allTrackables.get(3).setName("Red Alliance Wall");


        /**  Let all the trackable listeners know where the camera is.  */
    }

    private void tfodInit() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vLocalizer);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, new String[]{"ball", "block", "Duck"});
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }
    }

}

/*
        if (robot.vuforia.targetFound()) {
                telemetry.addLine("Target is Found");

                telemetry.addData("X", "position (%.2f)", robot.vuforia.getX());
                telemetry.addData("Y", "position (%.2f)", robot.vuforia.getY());
                telemetry.addData("Heading", "(%.2f) degrees", robot.vuforia.getHeading());
                }
                else {
                telemetry.addLine("No target found.");
                }
*/
