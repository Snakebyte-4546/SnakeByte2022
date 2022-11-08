package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.AutoMethods;

@Autonomous(name = "StrafeTest")
public class StrafeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AutoMethods robot = new AutoMethods();
        robot.ready(this);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            robot.Strafe(1, 650);
            sleep(30000);
        }
    }
}
