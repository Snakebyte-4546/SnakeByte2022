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
@Autonomous(name = "Spline Left Auto", group = "Score Auto")
public class SplineLeftAuto extends LinearOpMode {

    int tagOfInterest = 0;
    AutoMethods robot = new AutoMethods();

    @Override
    public void runOpMode() {

        // Roadrunner Setup
        robot.ready(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(90));
        Pose2d scorePose= new Pose2d(-29, -5, Math.toRadians(220));
        Pose2d pickupPose = new Pose2d(-60, -12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);


        // Trajectory setup
        TrajectorySequence preloadToGoal = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    scorePrime();
                })
                .splineToSplineHeading(new Pose2d(-29, -5, Math.toRadians(220)), Math.toRadians(62))
                .build();

        TrajectorySequence conePickup = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .addDisplacementMarker(() -> {
                    rest();
                })
                .splineToSplineHeading(pickupPose, Math.toRadians(180))
                .build();

        TrajectorySequence coneScore = drive.trajectorySequenceBuilder(conePickup.end())
                .addDisplacementMarker(() -> {
                    scorePrime();
                })
                .splineToSplineHeading(scorePose, Math.toRadians(40))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(coneScore.end())
                .lineToLinearHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(coneScore.end())
                .splineToSplineHeading(new Pose2d(-58, -11, Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(coneScore.end())
                .lineToLinearHeading(new Pose2d(-36, -11, Math.toRadians(180)))
                .strafeTo(new Vector2d(-12, -11))
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

        scorePrime();
        drive.followTrajectorySequence(preloadToGoal);
        score();

        scorePrime();
        drive.followTrajectorySequence(conePickup);
        robot.claw(false);
        drive.followTrajectorySequence(coneScore);
        score();
        scorePrime();
        rest();

        if(tagOfInterest == 1){
            drive.followTrajectorySequence(park1);
        } else if(tagOfInterest == 2){
            drive.followTrajectorySequence(park2);
        } else if(tagOfInterest == 3){
            drive.followTrajectorySequence(park3);
        }
    }

    private void scorePrime() {
        robot.moveLift(1, 4000);
        robot.moveFourBar(680);}

    private void score() {
        robot.moveFourBar(0);
        robot.moveFourBar(680);
        robot.claw(true);}

        private void rest() {
            robot.moveFourBar(0);
            robot.moveLift(1, 240);
            robot.claw(true);

        }
    }
