package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.robot.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.robot.AprilTags;
import org.firstinspires.ftc.teamcode.util.robot.AutoMethods;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Low Score Left Auto", group = "Score Auto")
public class LowLeftScoreAuto extends LinearOpMode {
    int tagOfInterest = 0;
    AutoMethods robot = new AutoMethods();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.ready(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence preloadToGoal = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    robot.moveLift(1,4000);
                })
                .lineTo(new Vector2d(-35,-40))
                .lineTo((new Vector2d(-51,-29)))
                .build();
        TrajectorySequence backUp = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .lineTo(new Vector2d(-51,-36))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(backUp.end())
                .lineTo(new Vector2d(-60,-36))
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(backUp.end())
                .lineTo(new Vector2d(-36, -36))
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(backUp.end())
                .lineTo(new Vector2d(-12,-36))
                .build();

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
        drive.followTrajectorySequence(preloadToGoal);
        sleep(1500);
        robot.claw(false);
        sleep(1500);
        drive.followTrajectorySequence(backUp);
        if(tagOfInterest == 1){
            drive.followTrajectorySequence(park1);
        } else if(tagOfInterest == 2){
            drive.followTrajectorySequence(park2);
        } else if(tagOfInterest == 3){
            drive.followTrajectorySequence(park3);
        }
    }
}
