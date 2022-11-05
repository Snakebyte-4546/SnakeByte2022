package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestServo", group = "Test")
public class TestServo extends OpMode {

    Servo right;
    Servo left;

    @Override
    public void init() {
        right = hardwareMap.servo.get("rightServo");
        left = hardwareMap.servo.get("leftServo");
        right.setPosition(1);
        left.setPosition(0);
    }

    @Override
    public void loop() {
        if(gamepad2.b){
            right.setPosition(.8);
            left.setPosition(.2);
        }
        if(gamepad2.a){
            right.setPosition(1);
            left.setPosition(0);
        }
    }
}
