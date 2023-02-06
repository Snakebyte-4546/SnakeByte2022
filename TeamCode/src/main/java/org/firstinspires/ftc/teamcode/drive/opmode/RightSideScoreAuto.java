package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.robot.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.robot.AprilTags;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


@Autonomous(name = "Right Side Auto", group = "Score Auto")
public class RightSideScoreAuto extends LinearOpMode {
    int tagOfInterest = 0;

    @Override
    public void runOpMode() {
        // Camera Setup
        AprilTags AprilTag = new AprilTags();
        AprilTagDetectionPipeline aprilTagDetectionPipeline = AprilTag.cameraSetup(this);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = AprilTag.getTag(aprilTagDetectionPipeline);
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                        tagOfInterest = tag.id;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addData("Tag Found:", tagOfInterest);
                }
            } else {
                telemetry.addLine("Tag not currently found: ");

                if (tagOfInterest == 0) {
                    telemetry.addLine("(no tags have been seen)");
                } else {
                    telemetry.addLine("\nBut the tag has been seen before, Tag ID: " + tagOfInterest);
                }
            }
            telemetry.update();
        }

        waitForStart();

        //Roadrunner Stuff
    }
}

