package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.AutoMethods;
import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;


@Autonomous(name = "Left Side Auto", group = "Score Auto")
public class LeftSideScore extends LinearOpMode {
    int tagOfInterest = 0;

    @Override
    public void runOpMode() {
        AutoMethods robot = new AutoMethods();
        AprilTagDetectionPipeline aprilTagDetectionPipeline = robot.cameraSetup(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap);
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
        robot.ready(this);
        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(90));
        Pose2d scorePose = new Pose2d(-29, -8, Math.toRadians(55));
        Pose2d stackPose = new Pose2d(-58,-11.8, Math.toRadians(180));

        drive.setPoseEstimate(startPose);
        TrajectorySequence scorePreload = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-35,-35))
                .splineTo(new Vector2d(-29,-8), Math.toRadians(55))
                .build();

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(scorePose)
                .lineTo(new Vector2d(-31,-10))
                .lineToLinearHeading(new Pose2d(-38,-11.8, Math.toRadians(180)))
                .lineTo(new Vector2d(-58,-11.8))
                .build();

        TrajectorySequence goToPole = drive.trajectorySequenceBuilder(stackPose)
                .lineToLinearHeading(new Pose2d(-40,-11.8, Math. toRadians(180)))
                .lineToLinearHeading(new Pose2d(-29,-8, Math.toRadians(55)))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(scorePose)
                .lineToLinearHeading(new Pose2d(-40,-11.8, Math. toRadians(180)))
                .lineTo(new Vector2d(-58,-11.8))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(scorePose)
                .lineToLinearHeading(new Pose2d(-35,-11.8, Math.toRadians(180)))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(scorePose)
                .lineToLinearHeading(new Pose2d(-10,-11.8, Math.toRadians(180)))
                .build();

        waitForStart();
        drive.followTrajectorySequence(scorePreload);
        robot.scoreHigh();
        //TODO: Fix stuck in stop()
        //TODO: Implement smart cycle timer
        for (int i = 0; i < 2; i++) {
            drive.followTrajectorySequence(goToStack);
            robot.pickStack(i);
            drive.followTrajectorySequence(goToPole);
            robot.scoreHigh();
        }
        if (tagOfInterest == 1) {
            drive.followTrajectorySequence(park1);
        } else if (tagOfInterest == 2) {
            drive.followTrajectorySequence(park2);
        } else if (tagOfInterest == 3) {
            drive.followTrajectorySequence(park3);
        }
    }
}
