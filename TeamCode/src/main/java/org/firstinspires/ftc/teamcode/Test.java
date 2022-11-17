package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.AutoMethods;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoMethods robot = new AutoMethods();
        robot.ready(this);
        waitForStart();
        robot.turnPDT(90,.5,2000);
    }
}
