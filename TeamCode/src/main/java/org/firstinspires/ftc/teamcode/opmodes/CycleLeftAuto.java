package org.firstinspires.ftc.teamcode.opmodes;

import android.provider.Telephony;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.AprilTags;
import org.firstinspires.ftc.teamcode.robot.AutoMethods;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorySequence.TrajectorySequence;
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
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        robot.ready(this);

        double  east = Math.toRadians(0);
        double  northEast = Math.toRadians(45);
        double  north = Math.toRadians(90);
        double  northWest = Math.toRadians(125);
        double  west = Math.toRadians(180);
        double  southWest = Math.toRadians(225);
        double  south = Math.toRadians(270);
        double  southEast = Math.toRadians(295);

        Pose2d start_pose      = new Pose2d(-35, -64, north);
        Vector2d start_vector  = new Vector2d(-35,-64);

        Pose2d scoreHigh_pose  = new Pose2d(-29, -4.4, southWest);
        Vector2d scoreHigh_vector  = new Vector2d(-29, -5);

        Pose2d pickup_pose     = new Pose2d(-55, -12, west);
        Vector2d pickup_vector = new Vector2d(-60, -12);

        Pose2d neutral_pose      = new Pose2d(-35, -12,west);
        Vector2d neutral_vector = new Vector2d(-35, -12);
        drive.setPoseEstimate(start_pose);

        // Trajectory setup

        TrajectorySequence startToPickup = drive.trajectorySequenceBuilder(start_pose)

                // from stat to neutral
                .lineTo(neutral_vector)
                .addDisplacementMarker(5, () -> {
                    prime();})

                // from neutral to highGoal
                .turn(Math.toRadians(135))
                .lineToLinearHeading(scoreHigh_pose)
                // score
                .addDisplacementMarker(() -> {
                    score();})
                .waitSeconds(0)

                // from highGoal to neutral
                .lineTo(neutral_vector)
                .turn(Math.toRadians(-45))

                // from neutral to pickup
                .addSpatialMarker(new Vector2d(-35 -12), () -> {
                    restAtConeLevel(numCones);})
                .lineTo(pickup_vector)
                .build();


        TrajectorySequence pickupFromNeutral = drive.trajectorySequenceBuilder(neutral_pose)
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


        TrajectorySequence scoreHigh = drive.trajectorySequenceBuilder(pickup_pose)

                //from pickup to neutral
                .lineTo(neutral_vector)

                //to High and score
                .turn(Math.toRadians(45))
                .lineToLinearHeading(scoreHigh_pose)
                .addDisplacementMarker(() -> {
                    score();
                })
                .waitSeconds(3)

                //back to neutral
                .lineTo(neutral_vector)
                .turn(Math.toRadians(-45))

                //from neutral to pickup
                .lineTo(pickup_vector)
                .build();

        TrajectorySequence pickupToStart = drive.trajectorySequenceBuilder(pickup_pose)

                //from pickup to neutral
                .lineTo(neutral_vector)
                .lineTo(start_vector)
                .build();

        TrajectorySequence park1 = drive.trajectorySequenceBuilder(neutral_pose)
                .lineTo(new Vector2d(-58, -12))
                .build();

        TrajectorySequence park2 = drive.trajectorySequenceBuilder(neutral_pose)
                .lineTo(new Vector2d(-35.5, -12))
                .build();

        TrajectorySequence park3 = drive.trajectorySequenceBuilder(neutral_pose)
                .lineTo(new Vector2d(-12, -12))
                .build();

        TrajectorySequence neutralToStart = drive.trajectorySequenceBuilder(neutral_pose)
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-35, -64))
                .build();


        // Camera Setup
        AprilTags AprilTag = new AprilTags();

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = AprilTag.getTag(AprilTag.cameraSetup(this));

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
            //telemetry.addData("Fourbar Position:", robot.fourBar.getCurrentPosition());
            //telemetry.addData("Claw Position:", robot.claw.getPosition());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            /*
            hold();
            drive.followTrajectorySequence(startToPickup); //score first cone

            numCones--;
            drive.followTrajectorySequence(scoreHigh); //scores again on high
            drive.followTrajectorySequence(pickupToStart);
            */
            posesRunthrough();

            //drive.followTrajectorySequence();


            if (tagOfInterest == 1) {
                drive.followTrajectorySequence(park1);
            } else if (tagOfInterest == 2) {
                drive.followTrajectorySequence(park2);
            } else if (tagOfInterest == 3) {
                drive.followTrajectorySequence(park3);
            } else {
                //drive.followTrajectorySequence(neutralToStart);
            }
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
        //robot.killThreads();
    }








    private void intake() {
        //will eventually call alignment method and movement stuff
        hold();
    }

    private void prime() { //takes ~2300 ms
        robot.claw(false);
        robot.setFourBar("up");
        robot.setLift("high");
        sleep(2000);}
    private void score() {
        robot.setFourBar("back");
        sleep(700);
        robot.claw(true);
        sleep(700);
        robot.claw(false);
        restFromScore();}

    private void primeLow() {     //doesn't use fourbar for low goal
        robot.claw(false);
        robot.setLift("mid");}

    private void scoreLow() {     //doesn't use fourbar for low goal
        robot.moveLift(0);
        robot.claw(false);}

    private void rest() {
        robot.setFourBar("rest");
        robot.claw(true);
        robot.setLift("rest");}

    private void restAtConeLevel(int numCones) {
        switch (numCones) {
            case 0: {
                robot.setFourBar("rest");
                robot.claw(true);
                robot.moveLift(0);
                break;
            }

            case 1: {
                robot.setFourBar("rest");
                robot.claw(true);
                robot.moveLift(.1);
                break;
            }

            case 2: {
                robot.setFourBar("rest");
                robot.claw(true);
                robot.moveLift(.2);
                break;
            }

            case 3: {
                robot.setFourBar("rest");
                robot.claw(true);
                robot.moveLift(.3);
                break;
            }

            case 4: {
                robot.setFourBar("rest");
                robot.claw(true);
                robot.moveLift(.4);
                break;
            }

            case 5: {
                robot.setFourBar("rest");
                robot.claw(true);
                robot.moveLift(.5);
                break;
            }
        }
    }

    private void restFromScore() {
        robot.setFourBar("rest");
        sleep(1000);
        robot.claw(true);
        robot.setLift("rest");}

    private void hold() {
        robot.claw(false);
        robot.setLift("hold");
        robot.setFourBar("hold");}

    private static Pose2d poseMaker(Vector2d cord, double head) {
        return new Pose2d(cord.getX() + .01, cord.getY(), head);
    }


    private void posesRunthrough() {
        telemetry.addLine("Prime");
        telemetry.update();
        prime();
        sleep(2300);
        telemetry.addLine("score");
        telemetry.update();
        score();
        sleep(1800);
        telemetry.addLine("Prime");
        telemetry.update();
        sleep(1800);
        telemetry.addLine("Prime");
        telemetry.update();
        restFromScore();
        telemetry.addLine("Cone stack: 5");
        telemetry.update();
        restAtConeLevel(5);
        sleep(2000);
        telemetry.addLine("Cone stack: 4");
        telemetry.update();
        restAtConeLevel(4);
        sleep(2000);
        telemetry.addLine("Cone stack: 3");
        telemetry.update();
        restAtConeLevel(3);
        sleep(2000);
        telemetry.addLine("Cone stack: 2");
        telemetry.update();
        restAtConeLevel(2);
        sleep(2000);
        telemetry.addLine("Cone stack: 1");
        telemetry.update();
        restAtConeLevel(1);
        sleep(2000);
        telemetry.addLine("Intake");
        telemetry.update();
        intake();
        telemetry.addLine("Hold");
        telemetry.update();
        hold();
        sleep(2000);
    }

}
