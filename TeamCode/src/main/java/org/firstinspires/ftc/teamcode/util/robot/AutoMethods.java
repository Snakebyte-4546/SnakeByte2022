package org.firstinspires.ftc.teamcode.util.robot;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoMethods {
    public DcMotor lift;
    public Servo claw;
    public Servo fourBar;
    public int liftPos;
    public double fourBarPos;

    public AutoMethods(OpMode opMode){
        lift = opMode.hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw = opMode.hardwareMap.servo.get("claw");
        fourBar = opMode.hardwareMap.servo.get("fourBar");
        fourBar.setPosition(0.6);
        claw.setPosition(0.1);
    }

    public void moveLift(int ticks){
        liftPos = ticks;
        lift.setTargetPosition(liftPos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }
    public void setFourBar(double position){
        fourBarPos = position;
        fourBar.setPosition(position);
    }

    public void toPos(String code){
        if(code.equals("Stack5")){}
    }
}
