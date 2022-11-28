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
        robot.moveLift(1,700);
        robot.lift.setPower(0.05);
        sleep(100);
        robot.MoveInchEncoder(-.8,950);
        robot.Strafe(.8, 3500);
        //move lift and score
        //lower lift
        robot.Strafe(-.8, 500);
        robot.MoveInchEncoder(-.8,1900);
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
