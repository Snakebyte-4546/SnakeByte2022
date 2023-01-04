package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.robot.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.robot.AprilTags;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;


@Autonomous(name = "Left Side Auto", group = "Score Auto")
public class LeftSideScoreAuto extends LinearOpMode {
    int tagOfInterest = 0;

    @Override
    public void runOpMode() {
        // Roadrunner Setup
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence path = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, -15, Math.toRadians(90)))
                .splineToLinearHeading(new Pose2d(-23.5, -10, Math.toRadians(90)), Math.toRadians(90))

                .lineToLinearHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-30, -9, Math.toRadians(230)))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(new Pose2d(-58, -11, Math.toRadians(180)))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(path.end())
                .lineToLinearHeading(new Pose2d(-12, -11, Math.toRadians(180)))
                .build();

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

        while (!isStopRequested()) {
            drive.followTrajectorySequence(path);
            if(tagOfInterest == 1){
                drive.followTrajectorySequence(park1);
            } else if(tagOfInterest == 2){
                drive.followTrajectorySequence(park2);
            } else if(tagOfInterest == 3){
                drive.followTrajectorySequence(park3);
            }
        }


    }
}

