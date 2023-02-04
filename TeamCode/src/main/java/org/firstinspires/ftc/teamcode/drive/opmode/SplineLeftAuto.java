package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;
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
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(90));
        Pose2d scorePose= new Pose2d(-29, -5, Math.toRadians(220));
        Pose2d pickupPose = new Pose2d(-60, -12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);


        // Trajectory setup
        TrajectorySequence preloadToGoal = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    prime();
                })
                .strafeTo(new Vector2d(-35, -10.2))
                .turn(Math.toRadians(130), Math.toRadians(80), Math.toRadians(150))
                .strafeTo(new Vector2d(-28.5, -4.5))
                .build();


        /*TrajectorySequence conePickup = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .strafeTo(new Vector2d(-60, -12))
                .build();

        TrajectorySequence coneScore = drive.trajectorySequenceBuilder(conePickup.end())
                .addDisplacementMarker(() -> {
                    prime();
                })
                .splineToSplineHeading(scorePose, Math.toRadians(40))
                .build();*/

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .strafeTo(new Vector2d(-36, -12)) // neutral pos
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150)) // neutral heading
                .strafeTo(new Vector2d(-62, -12)) //park pos
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .strafeTo(new Vector2d(-36, -12)) // neutral pos
                .turn(Math.toRadians(-40), Math.toRadians(80), Math.toRadians(150)) // neutral heading
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .strafeTo(new Vector2d(-36, -12)) //neutral
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


        /*drive.followTrajectorySequence(preloadToGoal);
        score();
        prime();
        sleep(500);
        robot.claw(true);*/
        rest();
        sleep(2000);
        prime();
        //sleep(2000);
        score();
        sleep(1500);
        restFromScore();
        sleep(2000);


        /*restAtConeLevel(1);
        sleep(1000);
        robot.claw(true);
        restAtConeLevel(2);
        sleep(1000);
        robot.claw(true);
        restAtConeLevel(3);
        sleep(1000);
        robot.claw(true);
        restAtConeLevel(4);
        sleep(1000);
        robot.claw(true);
        restAtConeLevel(5);
        sleep(1000);
        robot.claw(true);

        hold();

        sleep(2000);
        rest();*/


        /*if(tagOfInterest == 1){
            drive.followTrajectorySequence(park1);
        } else if(tagOfInterest == 2){
            drive.followTrajectorySequence(park2);
        } else if(tagOfInterest == 3){
            drive.followTrajectorySequence(park3);
        }*/
    }

    private void prime() {
        robot.claw(false);
        robot.moveLift(4000);
        robot.moveFourBar(660);}
    private void score() {
        robot.moveLift(4000);
        robot.moveFourBar(1000);
        sleep(500);
        robot.claw(true);
        sleep(750);
        robot.claw(false);}
    private void primeLow() {     //doesn't use fourbar for low goal
        robot.claw(false);
        robot.moveLift(4000);}

    private void scoreLow() {     //doesn't use fourbar for low goal
        robot.moveLift(0);
        robot.claw(false);}

    private void rest() {
        robot.moveLift(0);
        robot.moveFourBar(0);
        robot.claw(true);}

    private void restAtConeLevel(int numCones) {
        switch (numCones) {
            case 0: {
                robot.moveLift(0);
                sleep(500);
                robot.claw(true);
                break;
            }

            case 1: {
                robot.moveLift(1000);
                sleep(500);
                robot.claw(true);
                break;
            }

            case 2: {
                robot.moveLift(1200);
                sleep(500);
                robot.claw(true);
                break;
            }

            case 3: {
                robot.moveLift(1400);
                break;
            }

            case 4: {
                robot.moveLift(1600);
                break;
            }

            case 5: {
                robot.moveLift(650);
                break;
            }
        }
    }

    private void restFromScore() {
        robot.moveFourBar(0);
        sleep(500);
        robot.moveLift(0);
        sleep(2000);
        robot.claw(true);}

    private void hold() {
        robot.claw(false);
        sleep(250);
        robot.moveLift(1300);
        robot.moveFourBar(0);}
    }
