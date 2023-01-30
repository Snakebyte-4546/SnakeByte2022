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
    private int numCones;

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

        Pose2d start_pose      = new Pose2d(-35, -64, north);
        Pose2d scoreHigh_pose  = new Pose2d(-29, -5, northEast);
        Pose2d pickup_pose     = new Pose2d(-60, -12, west);
        Vector2d pickup_vector = new Vector2d(-60, -12);
        Vector2d neutral_vector = new Vector2d(-35, -12);
        drive.setPoseEstimate(start_pose);

        //


        // Trajectory setup

        TrajectorySequence startToHigh = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    prime();
                })
                .lineTo(neutral_vector)
                .turn(Math.toRadians(-45))
                .lineToLinearHeading(scoreHigh_pose)
                .build();

        TrajectorySequence highToNeutral = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .lineTo(neutral_vector)
                .turn(Math.toRadians(135))
                .build();

        TrajectorySequence pickupNeutral = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addSpatialMarker(new Vector2d(-35 -12), () -> {
                    restAtConeLevel(numCones);
                })
                .lineTo(pickup_vector)
                .addDisplacementMarker(() -> {
                    intake();
                })
                .waitSeconds(1)
                .lineTo(neutral_vector)
                .build();

        TrajectorySequence pickupToNeutral = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .build();

        TrajectorySequence scoreHigh = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-135))
                .lineToLinearHeading(scoreHigh_pose)
                .addDisplacementMarker(() -> {
                    score();
                })
                .waitSeconds(4)
                .lineTo(neutral_vector)
                .turn(Math.toRadians(135))
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-58, -12))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-12, -12))
                .build();

        TrajectorySequence neutralToStart = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(-35, -64))
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

        drive.followTrajectorySequence(highToNeutral);
        drive.followTrajectorySequence(pickupNeutral);
        numCones--;
        drive.followTrajectorySequence(scoreHigh);



        //drive.followTrajectorySequence();


        if (tagOfInterest == 1) {
            drive.followTrajectorySequence(park1);
        } else if (tagOfInterest == 2) {
            drive.followTrajectorySequence(park2);
        } else if (tagOfInterest == 3) {
            drive.followTrajectorySequence(park3);
        }
        else {
            drive.followTrajectorySequence(neutralToStart);
        }
    }

    private void intake() {
        //will eventually call alignment method and movement stuff
        hold();
    }

    private void prime() {
        robot.claw(false);
        robot.moveLift(1, 4000);
        robot.moveFourBar(660);}
    private void score() {
        robot.moveLift(1, 4000);
        robot.moveFourBar(1000);
        sleep(500);
        robot.claw(true);
        sleep(1000);
        robot.claw(false);
        prime();
        sleep(750);
        restFromScore();}

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
        sleep(1800);
        robot.claw(true);}

    private void hold() {
        robot.claw(false);
        sleep(250);
        robot.moveLift(1, 400);
        robot.moveFourBar(150);}

    private static Pose2d poseMaker(Vector2d cord, double head) {
        return new Pose2d(cord.getX() + .01, cord.getY(), head);
    }
}
