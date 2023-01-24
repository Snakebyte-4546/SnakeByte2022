package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OneDriver", group = "TeleOp")
public class OneDriverTeleOp extends OpMode {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;
    public DcMotor lift;
    public DcMotor lift2;
    public DcMotor fourbar;
    public Servo claw;
    public BNO055IMU imu;

    public int liftPos = 0;
    public int lift2Pos = 0;
    public int fBPos = 0;
    public double clawOpen = 1;
    public double clawClose = 0;
    public boolean isClosed = true;
    public double liftPower = 1;
    public double slowPower = 0.5;
    public double fourbarPower = .1;
    public boolean frontorBack = true;
    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        lift = hardwareMap.dcMotor.get("lift");
        lift2 = hardwareMap.dcMotor.get("lift2");
        claw = hardwareMap.servo.get("claw");
        fourbar = hardwareMap.dcMotor.get("4bar");
        fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourbar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double multiplier = 1;
            if(gamepad1.left_bumper){
                frontorBack = !frontorBack;
                multiplier *= -1;
            }
            double FLP = (y - x + rx) / max * multiplier;
            double BLP = (y + x + rx) / max * multiplier;
            double FRP = (y + x - rx) / max * multiplier;
            double BRP = (y - x - rx) / max * multiplier;
            fl.setPower(FLP);
            fr.setPower(FRP);
            bl.setPower(BLP);
            br.setPower(BRP);

        } else {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }

        if(Math.abs(gamepad1.right_stick_y) > 0.1) {
            if (gamepad1.right_trigger > .2) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(gamepad1.right_trigger);
                lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift2.setPower(gamepad1.right_trigger);
            } else if(gamepad1.left_trigger > .2){
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(-gamepad1.right_trigger);
                lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift2.setPower(-gamepad1.right_trigger);
            }
            liftPos = lift.getCurrentPosition();
            lift2Pos = lift2.getCurrentPosition();
        } else {
            lift.setTargetPosition(liftPos);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
            lift2.setTargetPosition(lift2Pos);
            lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift2.setPower(liftPower);
        }
        try {
            claw(this);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(gamepad1.dpad_up){
            fourbar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fourbar.setPower(.6);
        } else if(gamepad1.dpad_down){
            fourbar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fourbar.setPower(-.6);
        } else{
            fourbar.setPower(0);
        }
    }
    public void claw(OpMode opMode) throws InterruptedException {
        if (opMode.gamepad1.right_bumper) {
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
}

