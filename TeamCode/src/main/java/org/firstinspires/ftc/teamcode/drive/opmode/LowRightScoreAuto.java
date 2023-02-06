package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.robot.AutoMethods;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Low Score Right Auto", group = "Score Auto")
public class LowRightScoreAuto extends LinearOpMode {
    int tagOfInterest = 0;
    AutoMethods robot = new AutoMethods();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.ready(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence preloadToGoal = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    robot.moveLift(2000);
                })
                .lineTo(new Vector2d(-35,-40))
                .lineTo((new Vector2d(-24,-28)))
                .build();
        TrajectorySequence backUp = drive.trajectorySequenceBuilder(preloadToGoal.end())
                .lineTo(new Vector2d(-51,-40))
                .build();
        TrajectorySequence park1 = drive.trajectorySequenceBuilder(backUp.end())
                .lineTo(new Vector2d(-64,-40))
                .build();
        TrajectorySequence park2 = drive.trajectorySequenceBuilder(backUp.end())
                .lineTo(new Vector2d(-35,-40))
                .build();
        TrajectorySequence park3 = drive.trajectorySequenceBuilder(backUp.end())
                .lineTo(new Vector2d(-10,-40))
                .build();
        waitForStart();
        drive.followTrajectorySequence(preloadToGoal);
        sleep(1500);
        robot.claw(false);
        sleep(1500);
        if(tagOfInterest == 1){
            drive.followTrajectorySequence(park1);
        } else if(tagOfInterest == 2){
            drive.followTrajectorySequence(park2);
        } else if(tagOfInterest == 3){
            drive.followTrajectorySequence(park3);
        }
    }

}