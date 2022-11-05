package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MasterClass;

public class AutoMethods {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor lift;


    public void ready(LinearOpMode auto){
        fl = auto.hardwareMap.dcMotor.get("fl");
        fr = auto.hardwareMap.dcMotor.get("fr");
        bl = auto.hardwareMap.dcMotor.get("bl");
        br = auto.hardwareMap.dcMotor.get("br");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        lift = auto.hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMotorPower(double power){
        fl.setPower(power);
        br.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
    }

    public void moveLift(double speed, String position){
        double ticks = 0;
        if(position.equals("high")){
            ticks = 500;
        } else if(position.equals(("medium"))){
            ticks = 400;
        } else {
            ticks = 300;
        }
        while(Math.abs(lift.getCurrentPosition()) < ticks){
            lift.setPower(speed);
        }
        lift.setPower(0);
    }

    public void MoveInchEncoder(double speed, double ticks){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(bl.getCurrentPosition()) < ticks){
            setMotorPower(speed);
        }
        setMotorPower(0);
    }

    public void Strafe(double speed, boolean direction, double ticks){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(Math.abs(bl.getCurrentPosition()) < ticks) {
            if (direction) {
                fl.setPower(-speed);
                bl.setPower(speed);
                fr.setPower(speed);
                br.setPower(-speed);
            } else {
                fl.setPower(speed);
                bl.setPower(-speed);
                fr.setPower(-speed);
                br.setPower(speed);
            }
        }
        setMotorPower(0);
    }

}
