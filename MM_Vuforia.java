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
    private MM_OpMode opMode;  // Give us access to the calling OpMode

    //define constants for field navigation
    private static final String VUFORIA_KEY = "AZ5woGn/////AAABmSDumo9pA0BDovmvaV5gG7wLT6ES1QrKcI14JsHiEtQ7Gb6e+KM8ILBQGt8hjfHFNwKixlUDQ6vuz0AdKiYelGz5KcfJ9UV4xCMuDxDGvzOqYIS46QLHeFtsx4c4EP5o5a+H4ZM4crit1cva6avYORJXAH4EYCNluvawI+qm7qOru223kxOmNw83qfl17h9ASLtxxZuZ6OiAnQEq0OsSJf5n43QzVRFI55ZYdVAq+7bSeBEMptf1ZbrzvAZWnq8diTq+ojaADlkeZloub6tSLn4OqqbVtnjk65dNVejK2nTY1y7j7v0BQAkqc0w6oMkg30ynxOoyGid1xjSDDEaS1DvbVjQO0ODZZ4O9v6C30dtQ";
    private static final float MM_PER_INCH = 25.4f;
    private static final float MM_TARGET_HEIGHT = 6 * MM_PER_INCH;
    private static final float HALF_FIELD = 72 * MM_PER_INCH;
    private static final float HALF_TILE = 12 * MM_PER_INCH;
    private static final float ONE_AND_HALF_TILE = 36 * MM_PER_INCH;

    //declare webcam & instance variables
    private WebcamName webcam = null;
    private ElapsedTime runtime = new ElapsedTime();

    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vLocalizer = null;
    private VuforiaTrackables targets   = null ;
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

        while ((runtime.seconds() < 1.5) && duckLeftPixel < 0) {
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

        if  (duckLeftPixel < 0) {
            duckPosition = 3;
        } else if (duckLeftPixel > 220) {
            duckPosition = 2;
        } else {
            duckPosition = 1;
        }
        return duckPosition;
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
        return lastLocation.getTranslation().get(0) / MM_PER_INCH;
    }

    public double getY() {
        return lastLocation.getTranslation().get(1) / MM_PER_INCH;
    }

    public double getHeading() {
        return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES).thirdAngle;
    }

    private void vuforiaInit() {
        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcam = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcam;  // We indicate which camera we wish to use.
        parameters.useExtendedTracking = false;  // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.

        vLocalizer = ClassFactory.getInstance().createVuforia(parameters);  //  Instantiate the Vuforia engine

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        targets = vLocalizer.loadTrackablesFromAsset("FreightFrenzy");
        allTrackables.addAll(targets);

        // Name and locate each trackable object
        assignTargetPosition(0, "Blue Storage",       -HALF_FIELD, ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, 90, 0, 90);
        assignTargetPosition(1, "Blue Alliance Wall", HALF_TILE, HALF_FIELD, MM_TARGET_HEIGHT, 90, 0, 0);
        assignTargetPosition(2, "Red Storage",        -HALF_FIELD, -ONE_AND_HALF_TILE, MM_TARGET_HEIGHT, 90, 0, 90);
        assignTargetPosition(3, "Red Alliance Wall", HALF_TILE,  -HALF_FIELD, MM_TARGET_HEIGHT, 90, 0, 180);

        final float CAMERA_FORWARD_DISPLACEMENT  = 9.0f * MM_PER_INCH;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 10.0f * MM_PER_INCH;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 4.3f * MM_PER_INCH;   // eg: Enter the left distance from the center of the robot to the camera lens

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

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, new String[] {"ball", "block", "Duck"});
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0 / 9.0);
        }
    }

    private void assignTargetPosition(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
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
