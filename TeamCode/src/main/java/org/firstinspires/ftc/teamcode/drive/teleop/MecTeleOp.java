package org.firstinspires.ftc.teamcode.drive.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.TeleLib;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class MecTeleOp extends OpMode {
    TeleLib robot;

    @Override
    public void init() {
        robot = new TeleLib(this);
    }

    @Override
    public void loop() {
        robot.drivetrain(this);
        robot.lift(this);
        robot.claw(this);
    }

    @Override
    public void stop() {
        robot.kill();
    }
}


