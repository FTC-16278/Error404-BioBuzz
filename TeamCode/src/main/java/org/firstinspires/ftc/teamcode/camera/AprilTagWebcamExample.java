package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@Autonomous
public class AprilTagWebcamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    List<AprilTagDetection> detectedTags;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        detectedTags = aprilTagWebcam.getDetectedTags();

        for (AprilTagDetection detection : detectedTags) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                AprilTagSingleDetection id30 = aprilTagWebcam.getSingleTagBySpecificId(singleDet, 30);
                aprilTagWebcam.displayDetectionTelemetry(id30);
            } else { // cluster detection
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                AprilTagClusterDetection percent70 = aprilTagWebcam.getClusterTagByPercentage(clusterDet, 70);
                aprilTagWebcam.displayDetectionTelemetry(percent70);
            }
        }
    }
}
