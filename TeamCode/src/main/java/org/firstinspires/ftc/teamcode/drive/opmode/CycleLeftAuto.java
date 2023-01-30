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

@Autonomous(name = "Cycle Auto Left", group = "Score Auto")
public class CycleLeftAuto extends LinearOpMode {

    int tagOfInterest = 0;
    AutoMethods robot = new AutoMethods();

    @Override
    public void runOpMode() {

        // Roadrunner Setup
        robot.ready(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double  east = Math.toRadians(0);
        double  northEast = Math.toRadians(45);
        double  north = Math.toRadians(90);
        double  northWest = Math.toRadians(125);
        double  west = Math.toRadians(180);
        double  southWest = Math.toRadians(205);
        double  south = Math.toRadians(270);
        double  southEast = Math.toRadians(295);

        Pose2d startPose      = new Pose2d(-35, -64, north);
        Pose2d scorePose      = new Pose2d(-29, -5, northWest);
        Pose2d pickupPose     = new Pose2d(-60, -12, east);
        Vector2d pickupVector = new Vector2d(-60, -12);
        Vector2d nutralVector = new Vector2d(-35 -12);
        drive.setPoseEstimate(startPose);

        //


        // Trajectory setup
        TrajectorySequence startToHigh = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    prime();
                })
                .strafeTo(nutralVector)
                .turn(Math.toRadians(130), Math.toRadians(80), Math.toRadians(150))
                .lineToLinearHeading(scorePose)
                .build();

        TrajectorySequence goalToNuteral = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    rest();
                })
                .strafeTo(nutralVector)
                .turn(Math.toRadians(130), Math.toRadians(80), Math.toRadians(150))
                .build();

        TrajectorySequence nuteralToPickup = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(pickupVector)
                .build();

        TrajectorySequence pickupToNuteral = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(nutralVector)
                .build();

        TrajectorySequence nuteralToHigh = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    prime();
                })
                .splineToSplineHeading(scorePose, Math.toRadians(40))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(nuteralToHigh.end())
                .strafeTo(new Vector2d(-36, -12))
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                .strafeTo(new Vector2d(-58, -12))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(nuteralToHigh.end())
                .strafeTo(new Vector2d(-36, -12))
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(nuteralToHigh.end())
                .strafeTo(new Vector2d(-35, -12))
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


        drive.followTrajectorySequence(startToHigh);
        score();
        sleep(500);
        robot.claw(false);
        prime();
        sleep(750);
        rest();
        robot.claw(false);

        drive.followTrajectorySequence(goalToNuteral);
        drive.followTrajectorySequence(nuteralToPickup);
        drive.followTrajectorySequence(pickupToNuteral);
        drive.followTrajectorySequence(nuteralToHigh);
        //drive.followTrajectorySequence();


        if (tagOfInterest == 1) {
            drive.followTrajectorySequence(park1);
        } else if (tagOfInterest == 2) {
            drive.followTrajectorySequence(park2);
        } else if (tagOfInterest == 3) {
            drive.followTrajectorySequence(park3);
        }
    }

    private void prime() {
        robot.claw(false);
        robot.moveLift(1, 4000);
        robot.moveFourBar(660);}
    private void score() {
        robot.moveLift(1, 4000);
        robot.moveFourBar(1000);
        sleep(300);
        robot.claw(true);
        sleep(750);
        robot.claw(false);}

    private void primeLow() {     //doesn't use fourbar for low goal
        robot.claw(false);
        robot.moveLift(1, 4000);}

    private void scoreLow() {     //doesn't use fourbar for low goal
        robot.moveLift(1, 0);
        robot.claw(false);}

    private void rest() {
        robot.moveLift(1, 0);
        robot.moveFourBar(0);
        robot.claw(true);}

    private void restAtConeLevel(int numCones) {
        switch (numCones) {
            case 0: {
                robot.moveFourBar(0);
                sleep(500);
                robot.claw(true);
                break;
            }

            case 1: {
                robot.moveFourBar(110);
                sleep(500);
                robot.claw(true);
                break;
            }

            case 2: {
                robot.moveFourBar(140);
                sleep(500);
                robot.claw(true);
                break;
            }

            case 3: {
                robot.moveFourBar(170);
                break;
            }

            case 4: {
                robot.moveFourBar(200);
                break;
            }

            case 5: {
                robot.moveFourBar(230);
                break;
            }
        }
    }

    private void restFromScore() {
        robot.moveFourBar(0);
        sleep(500);
        robot.moveLift(1, 0);
        sleep(2000);
        robot.claw(true);}

    private void hold() {
        robot.claw(false);
        sleep(250);
        robot.moveLift(1, 400);
        robot.moveFourBar(150);}
}
