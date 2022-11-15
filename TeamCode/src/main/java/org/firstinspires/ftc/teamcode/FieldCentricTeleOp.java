package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.TeleLib;

@TeleOp(name = " Field Centric TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {
    TeleLib robot;

    @Override
    public void init() {
        robot = new TeleLib(this);
    }

    @Override
    public void loop() {
        robot.FieldCentricDriveTrain(this);
        try {
            robot.lift(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void stop() {
        robot.kill();
    }
}



