package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

@Autonomous
public class AprilTagWebcamExample extends OpMode {
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    AprilTagSingleDetection singleTagIdFound;
    AprilTagClusterDetection percentClusterFound;
    boolean USE_CLUSTER_TAG = true;
    int SINGLE_TAG_ID = 30;
    int CLUSTER_TAG_PERCENT = 70;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();

        if (!USE_CLUSTER_TAG) {
            singleTagIdFound = aprilTagWebcam.getSingleTagBySpecificId(SINGLE_TAG_ID);
            aprilTagWebcam.displayDetectionTelemetry(singleTagIdFound);
        } else {
            percentClusterFound = aprilTagWebcam.getClusterTagByPercentage(CLUSTER_TAG_PERCENT);
            aprilTagWebcam.displayDetectionTelemetry(percentClusterFound);
        }
    }
}
