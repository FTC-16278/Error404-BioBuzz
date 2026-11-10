package org.firstinspires.ftc.teamcode.mechanisms;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280, 720));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG); // stream format for robotic vision
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", detectedTags.size());
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedTag) {
        if (detectedTag != null) {
            if (detectedTag instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detectedTag;

                if (singleDet.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedTag.ftcPose.x, detectedTag.ftcPose.y, detectedTag.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedTag.ftcPose.pitch, detectedTag.ftcPose.roll, detectedTag.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedTag.ftcPose.range, detectedTag.ftcPose.bearing, detectedTag.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                }
            } else {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detectedTag;
                telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedTag.ftcPose.x, detectedTag.ftcPose.y, detectedTag.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedTag.ftcPose.pitch, detectedTag.ftcPose.roll, detectedTag.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedTag.ftcPose.range, detectedTag.ftcPose.bearing, detectedTag.ftcPose.elevation));
            }
        }
    }

    public AprilTagSingleDetection getSingleTagBySpecificId(AprilTagSingleDetection detection, int id) {
        if (detection.id == id)
            return detection;
        else
            return null;
    }

    public AprilTagClusterDetection getClusterTagByPercentage(AprilTagClusterDetection detection, int percent) {
        if (detection.percentClusterFound >= percent)
            return detection;
        else
            return null;
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
