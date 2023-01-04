package org.firstinspires.ftc.teamcode.util.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class TeleLib {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;
    public DcMotor lift;
    public Servo fourbar;
    public Servo claw;
    public BNO055IMU imu;

    public int liftPos = 0;
    public int fBPos = 0;
    public double clawOpen = .8;
    public double clawClose = .1;
    public boolean isClosed = true;
    public double liftPower = 1;
    public double slowPower = 0.5;

    public TeleLib(OpMode opMode) {
        fl = opMode.hardwareMap.dcMotor.get("fl");
        fr = opMode.hardwareMap.dcMotor.get("fr");
        bl = opMode.hardwareMap.dcMotor.get("bl");
        br = opMode.hardwareMap.dcMotor.get("br");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        fourbar = opMode.hardwareMap.servo.get("4bar");
        claw = opMode.hardwareMap.servo.get("claw");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        setFourBar(1);
        claw.setPosition(clawClose);
    }

    public void drivetrain(OpMode opMode) {
        if (Math.abs(opMode.gamepad1.left_stick_x) > 0.1 || Math.abs(opMode.gamepad1.left_stick_y) > 0.1 || Math.abs(opMode.gamepad1.right_stick_x) > 0.1) {
            double x = -opMode.gamepad1.left_stick_x;
            double y = -opMode.gamepad1.left_stick_y;
            double rx = opMode.gamepad1.right_stick_x;
            double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLP = (y - x + rx) / max;
            double BLP = (y + x + rx) / max;
            double FRP = (y + x - rx) / max;
            double BRP = (y - x - rx) / max;

            if (opMode.gamepad1.right_trigger > 0.1) {
                fl.setPower(FLP);
                fr.setPower(FRP);
                bl.setPower(BLP);
                br.setPower(BRP);
            } else {
                fl.setPower(FLP * slowPower);
                fr.setPower(FRP * slowPower);
                bl.setPower(BLP * slowPower);
                br.setPower(BRP* slowPower);
            }
        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
    }

    public void FieldCentricDriveTrain(OpMode opMode) {
        if (Math.abs(opMode.gamepad1.left_stick_x) > 0.1 || Math.abs(opMode.gamepad1.left_stick_y) > 0.1 || Math.abs(opMode.gamepad1.right_stick_x) > 0.1) {
            double x = -opMode.gamepad1.left_stick_x;
            double y = opMode.gamepad1.left_stick_y * 1.1;
            double rx = opMode.gamepad1.right_stick_x;
            double botHeading = imu.getAngularOrientation().firstAngle - 1.5708;
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = (rotY - rotX + rx) / max;
            double backLeftPower = (rotY + rotX + rx) / max;
            double frontRightPower = (rotY - rotX - rx) / max;
            double backRightPower = (rotY + rotX - rx) / max;

            fl.setPower(frontLeftPower);
            bl.setPower(backLeftPower);
            fr.setPower(frontRightPower);
            br.setPower(backRightPower);
        }
    }

    public void claw(OpMode opMode) throws InterruptedException {
        if (opMode.gamepad2.right_bumper) {
            if (isClosed) {
                claw.setPosition(clawOpen);
                isClosed = false;
            } else {
                claw.setPosition(clawClose);
                isClosed = true;
            }
            while (opMode.gamepad2.right_bumper) {
                Thread.sleep(100);
            }
        }
        opMode.telemetry.addData("claw (isclosed):", isClosed);
        opMode.telemetry.update();
    }

    public void lift(OpMode opMode) {
        if(Math.abs(opMode.gamepad2.right_stick_y) > 0.1) {
            if (opMode.gamepad2.right_trigger > .2) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(opMode.gamepad2.right_stick_y * .5);
            } else {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(opMode.gamepad2.right_stick_y);
            }
            liftPos = lift.getCurrentPosition();
        } else {
            lift.setTargetPosition(liftPos);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }
    }

    public void fourBar(OpMode opMode) throws InterruptedException {
        if(opMode.gamepad2.a) {
            setFourBar(1);
        }
        if (opMode.gamepad2.b) {
            setFourBar(2);
        }
        if (opMode.gamepad2.y) {
            setFourBar(3);
        }
        if (opMode.gamepad2.x) {
            setFourBar(4);
        }
    }

    public void setFourBar(int position) {
        switch(position) {
            case 1: {
                fourbar.setPosition(.6);
                fBPos = 1;
                break;
            }
            case 2: {
                fourbar.setPosition(.4);
                fBPos= 4;
                break;
            }
            case 3: {
                fourbar.setPosition(.2);
                fBPos = 5;
                break;
            }
            case 4: {
                fourbar.setPosition(0);
                fBPos = 6;
                break;
            }
        }
    }

    public void kill() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        lift.setPower(0);
        claw.setPosition(clawOpen);
    }
}