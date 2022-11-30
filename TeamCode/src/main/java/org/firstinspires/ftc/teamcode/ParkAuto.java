package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
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


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory backward1 = drive.trajectoryBuilder(new Pose2d())
                .back(24)
                .build();
        Trajectory strafe1 = drive.trajectoryBuilder(backward1.end())
                .strafeLeft(24)
                .build();
        Trajectory strafe2 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(24)
                .build();
        Trajectory forward3 = drive.trajectoryBuilder(new Pose2d())
                .forward(24)
                .build();
        Trajectory strafe3 = drive.trajectoryBuilder(forward3.end())
                .strafeLeft(24)
                .build();
        waitForStart();
        if(isStopRequested()){
            return;
        }
        if(tagOfInterest == 1) {
                drive.followTrajectory(backward1);
                drive.followTrajectory(strafe1);
            } else if(tagOfInterest == 2) {
                drive.followTrajectory(strafe2);
            } else if (tagOfInterest == 3) {
                drive.followTrajectory(forward3);
                drive.followTrajectory(strafe3);
            }
            else {
                telemetry.clearAll();
                telemetry.addLine("FATAL ERROR: NO TAGS FOUND");
                telemetry.update();
            }
    }
}

