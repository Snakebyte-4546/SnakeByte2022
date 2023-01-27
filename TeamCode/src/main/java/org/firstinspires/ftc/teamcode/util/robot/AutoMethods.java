package org.firstinspires.ftc.teamcode.util.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AutoMethods {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;
    public DcMotor lift;

    public DcMotor fourBar;

    int ticks = 0;
    double tickstodegs = 5;
    public double fourbarPower = .6;
    public int fBPos = 0;

    public Servo claw;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    ElapsedTime PIDTimer = new ElapsedTime();
    PIDCoefficients testPID = new PIDCoefficients(0,0,0);
    double integral = 0;

    public BNO055IMU imu;
    Orientation angles = new Orientation();


    public AprilTagDetectionPipeline cameraSetup(LinearOpMode opMode) {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("Camera Error:", errorCode);
                opMode.telemetry.update();
            }
        });
        return aprilTagDetectionPipeline;
    }

    public ArrayList<AprilTagDetection> getTag(AprilTagDetectionPipeline pipeline) {
        return pipeline.getLatestDetections();
    }

    public void ready(LinearOpMode auto) {
        fl = auto.hardwareMap.dcMotor.get("fl");
        fr = auto.hardwareMap.dcMotor.get("fr");
        bl = auto.hardwareMap.dcMotor.get("bl");
        br = auto.hardwareMap.dcMotor.get("br");
        drivetrainSetup();
        resetEncoder();

        lift = auto.hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        claw = auto.hardwareMap.servo.get("claw");
        claw(false);

        fourBar = auto.hardwareMap.dcMotor.get("4bar");
        fourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetLiftEncoder();

        BNO055IMU imu = auto.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

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

    public void setFourBar(int position) {
        switch(position) {
            case 0: {
                fourBar.setTargetPosition(0);
                fourBar.setPower(fourbarPower);
                fBPos = 1;
                break;
            }
            case 1: {
                fourBar.setTargetPosition(400);
                fourBar.setPower(fourbarPower);
                fBPos= 2;
                break;
            }
            case 2: {
                fourBar.setTargetPosition(680);
                fourBar.setPower(fourbarPower);
                fBPos = 3;
                break;
            }
            case 3: {
                fourBar.setTargetPosition(960);
                fourBar.setPower(fourbarPower);
                fBPos = 4;
                break;
            }
        }
    }

    public int getHeight(String code){
        int returnHeight = 0;
        if(code.equals("high")){
            returnHeight = 4000;
        }
        else if(code.equals("mid")){
            returnHeight = 3480;
        }
        else if(code.equals("low")){
            returnHeight = 2000;
        }
        else if(code.equals("driving")){
            returnHeight = 800;
        }
        else if(code.equals("s5")){
            returnHeight = 230;
        } else if(code.equals("s4")){
            returnHeight = 200;
        } else if(code.equals("s3")){
            returnHeight = 170;
        } else if(code.equals("s2")){
            returnHeight = 140;
        } else if(code.equals("s1")){
            returnHeight = 110;
        }
        else{
            returnHeight = 0;
        }
        return returnHeight;
    }

    public void moveLift(double speed, int position) {
        int ticks = position;
        lift.setTargetPosition(-ticks);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(-speed);
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

    public void claw(boolean isOpen) {
        if(!isOpen){
            claw.setPosition(0);

        } else {
            claw.setPosition(1);
        }
    }

    public void resetEncoder() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void drivetrainSetup() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetLiftEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int encoderAVG() {
        int avg = Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition());
        return avg/2;
    }

    public void turnPID(double targetAngle, double firstAngle){
        double firstError = targetAngle - firstAngle;
        double error = firstError;
        double lastError = 0;
        while (error < targetAngle){
            error = getGyroYaw();
            double changeInError = lastError - error;
            integral += changeInError * PIDTimer.time();
            double derivative = changeInError / PIDTimer.time();
            double P = testPID.p * error;
            double I = testPID.i * integral;
            double D = testPID.d * derivative;
            fl.setPower(P + I + D);
            fr.setPower(-P + -I + -D);
            bl.setPower(P + I + D);
            br.setPower(-P + -I + -D);
            error = lastError;
            PIDTimer.reset();
        }

    }
    public void moveFourBar(int ticks){
        fourBar.setTargetPosition(ticks);
        fourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourBar.setPower(1);
    }

}

