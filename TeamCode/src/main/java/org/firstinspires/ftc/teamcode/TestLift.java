package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestLift", group = "Test")
public class TestLift extends OpMode {

    DcMotor lift;

    @Override
    public void init() {
        lift = hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad2.x){
            while(lift.getCurrentPosition() < 50){
                lift.setPower(1);
            }
            lift.setPower(0);
        }
        if(gamepad2.y){
            while(lift.getCurrentPosition() > 0){
                lift.setPower(-1);
            }
            lift.setPower(0);
        }
        if(gamepad2.dpad_up){
            while(lift.getCurrentPosition() > 400){
                lift.setPower(1);
            }
            lift.setPower(0);
        }
        if(gamepad2.dpad_down){
            while(lift.getCurrentPosition() > 200){
                lift.setPower(1);
            }
            lift.setPower(0);
        }
    }
}
