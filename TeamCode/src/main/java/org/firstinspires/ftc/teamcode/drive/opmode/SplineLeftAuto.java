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
                .strafeTo(new Vector2d(-35, -10.2))
                .turn(Math.toRadians(130), Math.toRadians(80), Math.toRadians(150))
                .strafeTo(new Vector2d(-28.5, -4.5))
                .build();

        TrajectorySequence conePickup = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .strafeTo(new Vector2d(-60, -12))
                .build();

        TrajectorySequence coneScore = drive.trajectorySequenceBuilder(conePickup.end())
                .addDisplacementMarker(() -> {
                    scorePrime();
                })
                .splineToSplineHeading(scorePose, Math.toRadians(40))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(coneScore.end())
                .strafeTo(new Vector2d(-36, -12))
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                .strafeTo(new Vector2d(-28.5, -12))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(coneScore.end())
                .strafeTo(new Vector2d(-36, -12))
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(coneScore.end())
                .strafeTo(new Vector2d(-36, -12))
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                .strafeTo(new Vector2d(-12, -12))
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
                    telemetry.addLine("\nBut tag has been saved, Tag ID: " + tagOfInterest);
                }
            }
            telemetry.addData("Fourbar Position:", robot.fourBar.getCurrentPosition());
            telemetry.addData("Claw Position:", robot.claw.getPosition());
            telemetry.update();
        }

        waitForStart();


        drive.followTrajectorySequence(preloadToGoal);
        score();
        sleep(500);
        robot.claw(false);

        scorePrime();
        sleep(750);
        rest();
        robot.claw(false);
        sleep(250);

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
        robot.moveFourBar(660);}

    private void score() {
        robot.moveLift(1, 4000);
        robot.moveFourBar(1000);
        sleep(300);
        robot.claw(true);
        sleep(500);}

        private void rest() {
            robot.moveLift(1, 0);
            robot.moveFourBar(0);
            robot.claw(true);

        }
    }
