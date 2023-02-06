package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;

public class AutoMethods {
    // Drivetrain Motors
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;

    // Scoring Assembly Motors
    public DcMotorEx lift;
    public DcMotor lift2;
    public DcMotorEx fourBar;

    // Servos
    public Servo claw;

    // Variables
    int ticks = 0;
    final double tickstodegs = 5;
    public final double fourbarPower = .6;
    public int fBPos = 0;

    // Camera Setup + PID
    OpenCvCamera camera;
    AprilTags.AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // IMU
    public BNO055IMU imu;
    Orientation angles = new Orientation();

    //  PID Setup
    CustomPID lift_PID;
    CustomPID fourBar_PID;

    //oneClass implementation
    double gravity = .05;
    double lift_targetPosition;
    double lift_errorTolerance = 40;
    double lift_deltaError;
    double lift_lastError = 0;
    double lift_integral = 0;
    ElapsedTime lift_PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static PIDCoefficients lift_PIDCOEFFS = new PIDCoefficients(7, .0002, 1);
    private final PIDCoefficients lift_pidGains = new PIDCoefficients(0, 0, 0);

    double fbar_targetPosition;
    double fbar_errorTolerance = 30;
    double fbar_deltaError;
    double fbar_integral = 0;
    double fbar_lastError = 0;
    ElapsedTime fbar_PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static PIDCoefficients fbar_PIDCOEFFS = new PIDCoefficients(2, .0008, 1);
    private final PIDCoefficients fbar_pidGains = new PIDCoefficients(0, 0, 0);

    // Setup and Initialization
    public void ready(LinearOpMode auto) {


        // Drivetrain Motors
        fl = auto.hardwareMap.get(DcMotorEx.class, "fl");
        fr = auto.hardwareMap.get(DcMotorEx.class, "fr");
        bl = auto.hardwareMap.get(DcMotorEx.class, "bl");
        br = auto.hardwareMap.get(DcMotorEx.class, "br");
        drivetrainSetup();


        // Scoring Assembly Motors
        lift = auto.hardwareMap.get(DcMotorEx.class, "lift");
        fourBar = auto.hardwareMap.get(DcMotorEx.class, "4bar");

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetEncoder();

        // Intake Servo
        claw = auto.hardwareMap.servo.get("claw");
        claw(false);

        // IMU
        BNO055IMU imu = auto.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        // PID controller
        /*lift_PID = new CustomPID();
        lift_PID.motorSetup(20, new PIDCoefficients(3, 0, 2));
        fourBar_PID = new CustomPID();
        fourBar_PID.motorSetup(10, new PIDCoefficients(1, 0, 10));
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());*/
    }

    //Fourbar Methods
    public void setFourBar(String position) {
        switch (position) {
            case "rest": {
                moveFourBar(.07);
                fBPos = 1;
                break;
            }
            case "hold": {
                moveFourBar(.05);
                fBPos = 2;
                break;
            }
            case "front": {
                moveFourBar(.4);
                fBPos = 3;
                break;
            }
            case "up": {
                moveFourBar(.7);
                fBPos = 4;
                break;
            }
            case "back": {
                moveFourBar(.96);
                fBPos = 4;
                break;
            }
        }
    }

    public void moveFourBar(double position){
        fbar_targetPosition = position * 950;
        fbarThread.start();
        //fourBar.setPower(0);
        //fourBar.setPower(0);
        //fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //fourBar.setTargetPosition(position);
        //fourBar.setPower(1);
    }

    public void setLift(String position) {
        switch (position) {
            case "rest": {
                moveLift(0);
                fBPos = 1;
                break;
            }
            case "hold": {
                moveLift(.3);
                fBPos = 2;
                break;
            }

            case "low": {
                moveLift(.4);
                fBPos = 3;
                break;
            }

            case "mid": {
                moveLift(.55);
                fBPos = 3;
                break;
            }
            case "high": {
                moveLift(.93);
                fBPos = 4;
                break;
            }
        }
    }

    // Lift positions and values
    //  rest | mid | up
    //       |    |
    public void moveLift(double position) {
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_targetPosition = position * 2230;
        liftThread.start();
        //lift.setPower(0);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //lift.setTargetPosition(position);
        //lift.setPower(1);
    }

    // Claw Method
    public void claw(boolean isOpen) {
        if(!isOpen){
            claw.setPosition(0);

        } else {
            claw.setPosition(1);
        }
    }

    // Backup (Non-Roadrunner) Drivetrain Methods
    public void resetEncoder() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drivetrainSetup() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public int encoderAVG() {
        int avg = Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition());
        return avg/2;
    }


    public void MoveInchEncoder(double speed, double ticks) {
        resetEncoder();
        while(Math.abs(encoderAVG()) < ticks){
            setMotorPower(speed);
        }
        setMotorPower(0);
    }

    public void Strafe(double speed, double ticks) {
        resetEncoder();
        while(Math.abs(encoderAVG()) < ticks) {
            fl.setPower(-speed);
            bl.setPower(speed);
            fr.setPower(speed);
            br.setPower(-speed);
        }
        setMotorPower(0);
    }

    public void rotation(double power, int degs) {
        resetEncoder();
        degs *= tickstodegs;
        while (encoderAVG() < degs) {
            fl.setPower(power);
            bl.setPower(power);
            fr.setPower(-power);
            br.setPower(-power);
        }
        setMotorPower(0);
    }

    public void turn(double power) {
        fl.setPower(power);
        bl.setPower(power);
        fr.setPower(-power);
        br.setPower(-power);
    }

    public double getGyroYaw() {
        updateGyroValues();
        return angles.firstAngle;
    }

    public void updateGyroValues() {
        angles = imu.getAngularOrientation();
    }

    public void setMotorPower(double power) {
        fl.setPower(power);
        br.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
    }


    public boolean liftAtTarget() {
        return (lift.getCurrentPosition() >= (lift_targetPosition - lift_errorTolerance) && lift.getCurrentPosition() <= (lift_targetPosition + lift_errorTolerance)) && !(lift_deltaError >= 25);
    }
    private void PIDLift() {
        lift_PIDTimer.reset();

        double currentPosition = lift.getCurrentPosition();
        double error = lift_targetPosition - currentPosition;
        //telemetry.addLine("Error:  " + error);
        //telemetry.addData("Error", error);

        lift_deltaError = error - lift_lastError;
        double derivative = lift_deltaError / lift_PIDTimer.time();

        lift_integral += error * lift_PIDTimer.time();

        lift_pidGains.kP = error * lift_PIDCOEFFS.kP;
        lift_pidGains.kI = lift_integral * lift_PIDCOEFFS.kI;
        lift_pidGains.kD = derivative * lift_PIDCOEFFS.kD;

        lift.setVelocity(lift_pidGains.kP + lift_pidGains.kI + lift_pidGains.kD);
        lift_lastError = error;
    }
    public boolean fbarAtTarget() {
        return (fourBar.getCurrentPosition() >= (fbar_targetPosition - fbar_errorTolerance) && fourBar.getCurrentPosition() <= (fbar_targetPosition + fbar_errorTolerance)) && !(fbar_deltaError >= 25);
    }
    public void PIDfBar() {
        fbar_PIDTimer.reset();

        double currentPosition = fourBar.getCurrentPosition();
        double error = fbar_targetPosition - currentPosition;
        //telemetry.addLine("Error:  " + error);
        //telemetry.addData("Error", error);

        fbar_deltaError = error - fbar_lastError;
        double derivative = fbar_deltaError / fbar_PIDTimer.time();

        fbar_integral += error * fbar_PIDTimer.time();

        fbar_pidGains.kP = error * fbar_PIDCOEFFS.kP;
        fbar_pidGains.kI = fbar_integral * fbar_PIDCOEFFS.kI;
        fbar_pidGains.kD = derivative * fbar_PIDCOEFFS.kD;

        fourBar.setVelocity((fbar_pidGains.kP + fbar_pidGains.kI + fbar_pidGains.kD)) ;
        fbar_lastError = error;
    }

    Thread liftThread = new Thread() {
        public void run() {
            while (!liftAtTarget() && isAlive()) {
                PIDLift();
            }
        }
    };
    Thread fbarThread = new Thread() {
        public void run() {
            while (!fbarAtTarget() && isAlive()) {
                PIDfBar();
            }
        }
    };

    public void killThreads() {
        liftThread.interrupt();
        fbarThread.interrupt();
    }


}

