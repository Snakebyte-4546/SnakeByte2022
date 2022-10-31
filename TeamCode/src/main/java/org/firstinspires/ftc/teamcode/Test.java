package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Test extends OpMode {

    Servo right;
    Servo left;

    @Override
    public void init() {
        right = hardwareMap.servo.get("rightServo");
        left = hardwareMap.servo.get("leftServo");
        right.setPosition(0);
        left.setPosition(1);
    }

    @Override
    public void loop() {

    }
}
