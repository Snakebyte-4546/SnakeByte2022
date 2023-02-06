package org.firstinspires.ftc.teamcode.util.robot;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;

public class AprilTagDetectionPipeline extends OpenCvPipeline {
    private long nativeApriltagPtr;
    private final Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    public AprilTagDetectionPipeline() {
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
    public void finalize() {
        if(nativeApriltagPtr != 0) {
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, 0.166, 578.272, 578.272, 402.145, 221.506);
        return input;
    }

    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }
}