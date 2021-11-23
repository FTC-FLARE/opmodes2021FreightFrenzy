package org.firstinspires.ftc.teamcode.opmodes2021FreightFrenzy;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class MM_Vuforia {
    private OpMode opMode;

    private static final String VUFORIA_KEY = "AZ5woGn/////AAABmSDumo9pA0BDovmvaV5gG7wLT6ES1QrKcI14JsHiEtQ7Gb6e+KM8ILBQGt8hjfHFNwKixlUDQ6vuz0AdKiYelGz5KcfJ9UV4xCMuDxDGvzOqYIS46QLHeFtsx4c4EP5o5a+H4ZM4crit1cva6avYORJXAH4EYCNluvawI+qm7qOru223kxOmNw83qfl17h9ASLtxxZuZ6OiAnQEq0OsSJf5n43QzVRFI55ZYdVAq+7bSeBEMptf1ZbrzvAZWnq8diTq+ojaADlkeZloub6tSLn4OqqbVtnjk65dNVejK2nTY1y7j7v0BQAkqc0w6oMkg30ynxOoyGid1xjSDDEaS1DvbVjQO0ODZZ4O9v6C30dtQ";
    //define constants
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    private ElapsedTime runtime = new ElapsedTime();

    //define classes and webcam
    
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vLocalizer = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    private boolean targetVisible = false;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

    private TFObjectDetector tfod;

    public MM_Vuforia(OpMode opMode) {
        this.opMode = opMode;
        vuforiaInit();
        tfodInit();

    }

    private void vuforiaInit() {
        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vLocalizer = ClassFactory.getInstance().createVuforia(parameters);

        targets = vLocalizer.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targets);

        // Name and locate each trackable object
        assignTargetPosition(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        assignTargetPosition(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        assignTargetPosition(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        assignTargetPosition(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);


        final float CAMERA_FORWARD_DISPLACEMENT  = 9.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 10.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 4.3f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }
        targets.activate();
    }

    private void tfodInit() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vLocalizer);

        String[] labels = {"Duck"};

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, labels);
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }
    }

/*    public void seeDuck() {
        if (tfod != null) {
            List<Recognition> tfodRecognitions = tfod.getRecognitions();
            if (!tfodRecognitions.isEmpty()) {
                Recognition recognition = tfodRecognitions.get(0);
                opMode.telemetry.addData("Duck Found left: %.03f", recognition.getLeft());
                opMode.telemetry.update();
            }

        }

    }*/

    public void findDuck() {

        runtime.reset();
        boolean duckFound = false;
        while ((runtime.seconds() < 3) && !duckFound)
            if (tfod != null) {
                List<Recognition> tfodRecognitions = tfod.getRecognitions();
                if (!tfodRecognitions.isEmpty()) {
                    Recognition recognition = tfodRecognitions.get(0);
                    opMode.telemetry.addData("Duck Found left: %.03f", recognition.getLeft());
                    opMode.telemetry.update();
                    duckFound = true;
                }

            }

    }

    private void assignTargetPosition(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    public void deactivateTargets() {
        targets.deactivate();
    }

    public void deactivateTfod() {
        tfod.deactivate();
    }

    public boolean targetFound() {

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getRobotLocation();
                lastLocation = robotLocationTransform;
                opMode.telemetry.addData("Image Found", trackable.getName());
                return true;
            }
        }
        return false;
    }

    public double getX() {
        return lastLocation.getTranslation().get(0) / mmPerInch;
    }

    public double getY() {
        return lastLocation.getTranslation().get(1) / mmPerInch;
    }

    public double getHeading() {
        return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
    }
}

