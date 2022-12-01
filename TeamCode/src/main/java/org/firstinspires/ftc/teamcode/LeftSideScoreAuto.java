package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.AutoMethods;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


@Autonomous(name = "Left Side Auto", group = "Score Auto")
public class LeftSideScoreAuto extends LinearOpMode {
    Pose2d startPose = new Pose2d(-30, -60, 0);
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

        TrajectorySequence score = drive.trajectorySequenceBuilder(new Pose2d(-34, -61, 0))
                .splineTo(new Vector2d(-13, -45), Math.toRadians(90))
                .splineTo(new Vector2d(-13, -30), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-12.5, -0, Math.toRadians(-180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-12.5, -12, Math.toRadians(-180)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-15, -12, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-56, -12))
                .build();

        waitForStart();
        if (!isStopRequested()){
            drive.followTrajectorySequence(score);
        }
    }
}

