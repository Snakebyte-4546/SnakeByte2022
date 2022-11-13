package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.AutoMethods;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


@Disabled
@Autonomous(name = "Left Side Auto", group = "Score Auto")
public class LeftSideScoreAuto extends LinearOpMode {
    int tagOfInterest = 0;

    @Override
    public void runOpMode() {
        // Camera Setup
        AutoMethods robot = new AutoMethods();
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

        robot.ready(this);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            if(tagOfInterest == 1) {
                robot.MoveInchEncoder(-.25,650);
                robot.Strafe(.25, 1050);
                robot.moveLift(.5, 1);
                sleep(200);
                robot.MoveInchEncoder(.25, 60);
                robot.clamp(false);
                sleep(200);
                robot.MoveInchEncoder(-.25, 60);
                robot.Strafe(-.25, 200);
            } else if(tagOfInterest == 2) {
                robot.Strafe(.25, 400);
                robot.moveLift(.5, 1);
                sleep(200);
                robot.MoveInchEncoder(.25,60);
                robot.clamp(true);
                sleep(200);
                robot.MoveInchEncoder(-.5,60);
                robot.Strafe(.25, 250);
            } else if (tagOfInterest == 3) {
                robot.Strafe(.25, 400);
                robot.moveLift(.5, 1);
                sleep(200);
                robot.MoveInchEncoder(.25,60);
                robot.clamp(false);
                sleep(200);
                robot.MoveInchEncoder(-.5,60);
                robot.Strafe(.25, 250);
                robot.MoveInchEncoder(1, 650);
            } else {
                telemetry.clearAll();
                telemetry.addLine("FATAL ERROR: NO TAGS FOUND");
                telemetry.update();
            }
            robot.setMotorPower(0);
        }
    }
}

