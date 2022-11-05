package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class AprilTagAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int tagOfInterest = 0;

    @Override
    public void runOpMode() {
        // Camera Setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        // Setup Camera Pipeline
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error:", errorCode);
                telemetry.update();
            }
        });

        while (!isStarted() && !isStopRequested()) {
            // Update with latest data
            ArrayList < AprilTagDetection > currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            // Check if tags are detected
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                // Ignoring all other tags that may be present
                for (AprilTagDetection tag: currentDetections) {
                    if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                        tagOfInterest = tag.id;
                        tagFound = true;
                        break;
                    }
                }
                // If a tag is found
                if (tagFound) { telemetry.addData("Tag Found:", tagOfInterest); }

                // Something is wrong
            } else {
                telemetry.addLine("Tag not currently found:");

                if (tagOfInterest == 0) {
                    telemetry.addLine("(no tags have been seen)");
                } else {
                    telemetry.addLine("\nBut the tag has been seen before,");
                }
            }
            telemetry.update();
        }

        telemetry.clear();
        if (tagOfInterest != 0) {
            telemetry.addData("Tag ID:", tagOfInterest);
        } else {
            telemetry.addLine("*** No tag available, it was never found during init ***");
        }
        telemetry.update();
    }
}