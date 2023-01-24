package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.robot.TeleLib;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {
    TeleLib robot;

    @Override
    public void init() {
        robot = new TeleLib(this);
    }

    @Override
    public void loop() {
        robot.drivetrain(this);
        robot.lift(this);
        try {
            robot.claw(this);
        } catch (InterruptedException e) {
            telemetry.addData("ERROR: ", e);
            telemetry.update();
        }
         robot.fourBar(this);


    }

    @Override
    public void stop() {
        robot.kill();
    }
}



