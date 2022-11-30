package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.AutoMethods;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


@Autonomous(name = "Right Side Auto", group = "Score Auto")
public class RightSideScoreAuto extends LinearOpMode {
    Pose2d startPose = new Pose2d(30, -60, 180);
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
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Trajectory score = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(12, 0, Math.toRadians(0)), Math.toRadians(85))
                .build();

        Trajectory tag3 = drive.trajectoryBuilder(score.end())
                .strafeRight(12)
                .build();

        Trajectory tag2 = drive.trajectoryBuilder(score.end())
                .strafeRight(12)
                .forward(23)
                .build();

        Trajectory tag1 = drive.trajectoryBuilder(score.end())
                .strafeRight(12)
                .forward(44)
                .build();

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            drive.followTrajectory(score);
            //Do Claw Action Here
            if(tagOfInterest == 1) {
                drive.followTrajectory(tag1);
            } else if(tagOfInterest == 2) {
                drive.followTrajectory(tag2);
            } else if (tagOfInterest == 3) {
                drive.followTrajectory(tag3);
            } else {
                telemetry.clearAll();
                telemetry.addLine("FATAL ERROR: NO TAGS FOUND");
                telemetry.update();
            }
        }
    }
}

