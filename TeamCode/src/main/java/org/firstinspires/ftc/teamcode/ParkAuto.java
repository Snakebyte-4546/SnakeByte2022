package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.AutoMethods;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


@Autonomous(name = "Park Auto", group = "Auto")
public class ParkAuto extends LinearOpMode {

    int tagOfInterest = 0;

    @Override
    public void runOpMode() {
        // Camera/Robot Setup
        AutoMethods robot = new AutoMethods();
        robot.ready(this);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = robot.cameraSetup(this);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = robot.getTag(aprilTagDetectionPipeline);

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

        if(tagOfInterest == 1) {
                robot.MoveInchEncoder(-1,950);
                robot.Strafe(1, 1800);
            } else if(tagOfInterest == 2) {
            robot.MoveInchEncoder(-1,75);
            robot.Strafe(1, 1800);
            } else if (tagOfInterest == 3) {
            robot.MoveInchEncoder(1,900);
            robot.Strafe(1, 1800);
            } else {
                telemetry.clearAll();
                telemetry.addLine("FATAL ERROR: NO TAGS FOUND");
                telemetry.update();
            }
    }
}

