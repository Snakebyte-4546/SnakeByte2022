package org.firstinspires.ftc.teamcode.util.robot;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TeleLib {
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

    public TeleLib(OpMode opMode) {
        fl = opMode.hardwareMap.dcMotor.get("fl");
        fr = opMode.hardwareMap.dcMotor.get("fr");
        bl = opMode.hardwareMap.dcMotor.get("bl");
        br = opMode.hardwareMap.dcMotor.get("br");
        lift = opMode.hardwareMap.dcMotor.get("lift");
        lift2 = opMode.hardwareMap.dcMotor.get("lift2");
        claw = opMode.hardwareMap.servo.get("claw");
        fourbar = opMode.hardwareMap.dcMotor.get("4bar");
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
        BNO055IMU imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        setFourBar(4);
        claw.setPosition(clawClose);
    }

    public void drivetrain(OpMode opMode) {
        if (Math.abs(opMode.gamepad1.left_stick_x) > 0.1 || Math.abs(opMode.gamepad1.left_stick_y) > 0.1 || Math.abs(opMode.gamepad1.right_stick_x) > 0.1) {
            double x = -opMode.gamepad1.left_stick_x;
            double y = -opMode.gamepad1.left_stick_y;
            double rx = opMode.gamepad1.right_stick_x;
            double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double multiplier = 1;
            if(opMode.gamepad1.left_bumper){
                frontorBack = !frontorBack;
                multiplier *= -1;
            }
            double FLP = (y + x + rx) / max * multiplier;
            double BLP = (y - x + rx) / max * multiplier;
            double FRP = (y + x - rx) / max * multiplier;
            double BRP = (y - x - rx) / max * multiplier;
            if (opMode.gamepad1.right_trigger > 0.1) {
                fl.setPower(FLP);
                fr.setPower(FRP);
                bl.setPower(BLP);
                br.setPower(BRP);
            } else if(opMode.gamepad1.left_trigger > 0.1){
                fl.setPower(FLP * slowPower * slowPower);
                fr.setPower(FRP * slowPower * slowPower);
                bl.setPower(BLP * slowPower * slowPower);
                br.setPower(BRP* slowPower * slowPower);
            } else{
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

    public void mechanism(OpMode opMode){
        lift(opMode);
        fourBar(opMode);
        fourBarPID(-177);
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
                lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift2.setPower(opMode.gamepad2.right_stick_y * .5);
            } else {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(opMode.gamepad2.right_stick_y);
                lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift2.setPower(opMode.gamepad2.right_stick_y);
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
    }

    public void fourBar(OpMode opMode){
        if(Math.abs(opMode.gamepad2.left_stick_y) > 0.1){
            fourbar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fourbar.setPower(opMode.gamepad2.left_stick_y);
        } else if(opMode.gamepad2.y){
            fourbar.setTargetPosition(-40);
            fourbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fourbar.setPower(1);
        } else {
            fourbar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fourbar.setPower(0);
        }

    }

    public void fourBarPID(int targetPos){
        double kD = 0;
        double kP = 1;
        double kI = 0;
        double reference = targetPos;
        double integralSum = 0;
        double lastError = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(Math.abs(fourbar.getCurrentPosition()) < reference){
            double encoderPos = fourbar.getCurrentPosition();
            double error = reference - (encoderPos * Math.signum(reference));
            double derivative = (error -  lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());
            double out = (kP * error) + (kI * integralSum) + (kD * derivative);
            fourbar.setPower(out);
            lastError =  error;
            timer.reset();
        }
    }

    public void setFourBar(int position) {
        switch(position) {
            case 1: {
                fourbar.setTargetPosition(0);
                fourbar.setPower(fourbarPower);
                fBPos = 1;
                break;
            }
            case 2: {
                fourbar.setTargetPosition(0);
                fourbar.setPower(fourbarPower);
                fBPos= 2;
                break;
            }
            case 3: {
                fourbar.setTargetPosition(0);
                fourbar.setPower(fourbarPower);
                fBPos = 3;
                break;
            }
            case 4: {
                fourbar.setTargetPosition(0);
                fourbar.setPower(fourbarPower);
                fBPos = 4;
                break;
            }
            case 5: {
                fourbar.setTargetPosition(0);
                fourbar.setPower(fourbarPower);
                fBPos = 5;
            }
        }
    }

    public void kill() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        lift.setPower(0);
        lift2.setPower(0);
        fourbar.setPower(0);
        claw.setPosition(clawOpen);
    }
}