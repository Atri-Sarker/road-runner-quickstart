package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

public class WebcamControl {

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    /**
     * COLOR SENSORS
     */
    public PredominantColorProcessor leftColorSensor, middleColorSensor, rightColorSensor;

    /**
     * Regions for color sensors
     */
    public ImageRegion leftRegion = ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1);
    public ImageRegion middleRegion = ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1);
    public ImageRegion rightRegion = ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1);

    // VISION PORTAL
    public VisionPortal portal;



    // Init Webcam
    public WebcamControl(HardwareMap hardwareMap) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the color processors
        leftColorSensor = new PredominantColorProcessor.Builder()
                .setRoi(leftRegion)
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        middleColorSensor = new PredominantColorProcessor.Builder()
                .setRoi(middleRegion)
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        rightColorSensor = new PredominantColorProcessor.Builder()
                .setRoi(rightRegion)
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        // CREATE THE VISION PORTAL WITH ALL PROCESSORS
        portal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .addProcessor(leftColorSensor)
                .addProcessor(middleColorSensor)
                .addProcessor(rightColorSensor)
                .setCameraResolution(new Size(640, 360))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    // Scan For Color Combination
    private String mapColorToCode(PredominantColorProcessor.Swatch swatch) {
        if (swatch == null) {
            return "?";
        }

        switch (swatch) {
            case ARTIFACT_GREEN:
                return "G";
            case ARTIFACT_PURPLE:
                return "P";
            case RED:
                return "R";
            case BLUE:
                return "B";
            case YELLOW:
                return "Y";
            case BLACK:
                return "K";
            case WHITE:
                return "W";
            default:
                return "?";
        }
    }

    // Scan For Color Combination
    public String scanArtifactColors() {

        // Get detected predominant colors from each region
        PredominantColorProcessor.Swatch left   = leftColorSensor.getAnalysis().closestSwatch;
        PredominantColorProcessor.Swatch middle = middleColorSensor.getAnalysis().closestSwatch;
        PredominantColorProcessor.Swatch right  = rightColorSensor.getAnalysis().closestSwatch;

        // Convert swatch to single letter code
        String leftCode   = mapColorToCode(left);
        String middleCode = mapColorToCode(middle);
        String rightCode  = mapColorToCode(right);

        // If any are unknown, return UNKNOWN
        if (leftCode.equals("?") || middleCode.equals("?") || rightCode.equals("?")) {
            return "UNKNOWN";
        }

        // Concatenate into pattern
        return leftCode + middleCode + rightCode;
    }

    // Scan For Motif Pattern April Tag
    public String decodeMotifFromID(int id) {
        switch (id) {
            case 21:
                return "GPP";
            case 22:
                return "PGP";
            case 23:
                return "PPG";
            default:
                return "UNKNOWN";
        }
    }

    //
    public String waitForDecodeMotif(double timeoutSeconds) {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < timeoutSeconds) {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection tag : currentDetections) {

                String motif = decodeMotifFromID(tag.id);

                if (!motif.equals("UNKNOWN")) {
                    // Disable April Tag Processor
                    portal.setProcessorEnabled(aprilTag, false);
                    return motif;     // return immediately when a valid motif tag is found
                }
            };

        }

        return "PGP";   // Random Motif
    }



}
