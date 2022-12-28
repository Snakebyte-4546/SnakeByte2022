package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AutoMethods;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutoMethods robot = new AutoMethods();
        robot.ready(this);
        waitForStart();
        robot.moveLift(0.5, "driving");
        sleep(5000);
        //pick cone up for straight
        
        /*
        robot.MoveInchEncoder(.5, 50);
        sleep(100);
        robot.moveLift(1,2000);
        robot.lift.setPower(0.05);
        robot.clamp(false);
         */
    }
}
