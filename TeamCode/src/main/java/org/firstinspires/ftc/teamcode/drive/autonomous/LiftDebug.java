package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.robot.AutoMethods;
import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;


@Autonomous()
public class LiftDebug extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoMethods robot = new AutoMethods();
        robot.ready(this);

        waitForStart();
        int pos = 4100;

        while (!isStopRequested()) {
            telemetry.addData("Height", robot.lift.getCurrentPosition());
            telemetry.addData("SetHeight", pos);
            telemetry.update();
            robot.lift.setTargetPosition(pos);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
        }
    }
}
