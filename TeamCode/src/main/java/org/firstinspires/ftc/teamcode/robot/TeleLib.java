package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleLib {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;.
    public DcMotor br;
    public DcMotor lift;
    public Servo claw;

    public int liftPos = 0;

    public TeleLib(OpMode opMode) {
        fl = opMode.hardwareMap.dcMotor.get("fl");
        fr = opMode.hardwareMap.dcMotor.get("fr");
        bl = opMode.hardwareMap.dcMotor.get("bl");
        br = opMode.hardwareMap.dcMotor.get("br");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);

        lift = opMode.hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = opMode.hardwareMap.servo.get("claw");
        claw.setPosition(1);

        resetEncoders();
        resetLiftEncoder();
    }

    public void resetEncoders() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetLiftEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drivetrain(OpMode opMode) {
        if(Math.abs(opMode.gamepad1.left_stick_x) > 0.1 || Math.abs(opMode.gamepad1.left_stick_y) > 0.1 || Math.abs(opMode.gamepad1.right_stick_x) > 0.1) {
            double x = -opMode.gamepad1.left_stick_x;
            double y = opMode.gamepad1.left_stick_y;
            double rx = -opMode.gamepad1.right_stick_x;
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLP = (y + x + rx) / denom;
            double BLP = (y - x + rx) / denom;
            double FRP = (y - x - rx) / denom;
            double BRP = (y + x - rx) / denom;

            if (opMode.gamepad1.right_trigger > 0.1) {
                fl.setPower(FLP * .35);
                fr.setPower(FRP * .35);
                bl.setPower(BLP * .35);
                br.setPower(BRP * .35);
            } else {
                fl.setPower(FLP);
                fr.setPower(FRP);
                bl.setPower(BLP);
                br.setPower(BRP);
            }
        }
        else{
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
    }

    public void lift(OpMode opMode) {
        if(Math.abs(opMode.gamepad2.left_stick_y) > 0.1) {
            if (opMode.gamepad2.right_trigger > .2) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(opMode.gamepad2.left_stick_y * .5);
            } else {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(opMode.gamepad2.left_stick_y);
            }
            liftPos = lift.getCurrentPosition();
        } else {
            lift.setTargetPosition(liftPos);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
        }
        if(opMode.gamepad2.a){
            liftPos = 700;
        }
        if(opMode.gamepad2.b){
            liftPos = 1050;
        }
        if(opMode.gamepad2.y){
            liftPos = 2000;
        }
        opMode.telemetry.addData("Lift target pos: ", liftPos);
        opMode.telemetry.addData("Lift current position: ", lift.getCurrentPosition());
        opMode.telemetry.update();
    }

    public void claw(OpMode opMode) {
        if (opMode.gamepad2.right_bumper) {
                claw.setPosition(1);
        } else if (opMode.gamepad2.left_bumper) {
            claw.setPosition(.5);
        }
    }

    public void kill() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        lift.setPower(0);
    }
}