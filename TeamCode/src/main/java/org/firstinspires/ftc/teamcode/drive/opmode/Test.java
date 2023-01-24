package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.robot.TeleLib;

@TeleOp(name = "Test", group = "TeleOp")
public class Test extends OpMode{
    TeleLib robot;
    @Override
    public void init() {
        robot = new TeleLib(this);
    }

    @Override
    public void loop() {
        telemetry.addData("fourBarPos", robot.fourbar.getCurrentPosition());
    }
}
