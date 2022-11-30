package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.AutoMethods;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "test")
public class testAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Trajectory forward10 = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();
        Trajectory strafe = drive.trajectoryBuilder(forward10.end())
                .strafeRight(10)
                .build();
        Trajectory back10 = drive.trajectoryBuilder(strafe.end())
                .back(10)
                .build();
        waitForStart();
        if(isStopRequested()){
            return;
        }
        drive.followTrajectory(forward10);
        drive.followTrajectory(strafe);
        drive.followTrajectory(back10);
    }
}
