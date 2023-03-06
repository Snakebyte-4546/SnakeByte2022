package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    int ticks = 0;
    double tickstodegs = 22.75;

    public Servo claw;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    ElapsedTime elapsedTime = new ElapsedTime();

    public BNO055IMU imu;
    Orientation angles;


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

    public double getGyroYaw() {
        updateGyroValues();
        return angles.firstAngle;
    }

    public void updateGyroValues() {
        angles = imu.getAngularOrientation();
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
        resetLiftEncoder();

        claw = auto.hardwareMap.servo.get("claw");
        clamp(true);

        BNO055IMU imu = auto.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

    }

    public void setMotorPower(double power) {
        fl.setPower(-power);
        br.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
    }


    public int getHeight(String code){
        int returnHeight = 0;
        if(code.equals("high")){
            returnHeight = 4960;
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
            returnHeight = 850;
        } else if(code.equals("s4")){
            returnHeight = 775;
        } else if(code.equals("s3")){
            returnHeight = 700;
        } else if(code.equals("s2")){
            returnHeight=625;
        } else if(code.equals("s1")){
            returnHeight=550;
        }
        return returnHeight;
    }

    public void moveLift(double speed, String position) {
        ticks = getHeight(position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(-ticks);
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
            fr.setPower(-speed);
            br.setPower(speed);
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



    public void turnPID(double degree, double time, double power){
        elapsedTime.reset();
        double previousTime = elapsedTime.milliseconds();
        double currentPosition = getGyroYaw();
        double previousError = degree- currentPosition;
        double plusMin = 0;
        while(elapsedTime.milliseconds() < time ){
            double currentTime = elapsedTime.milliseconds();
            double currentError = degree-currentPosition;
            double p = 1.1 * currentError;
            double i = 0;
            i += .8* (currentTime-previousTime);
            double d = 1 * (currentError - previousError) / (currentTime - previousTime);
            double speed = p + i + d;
            plusMin = Math.signum((currentError));
            fl.setPower(-plusMin * speed * power);
            fr.setPower(plusMin * speed * power);
            br.setPower(plusMin * speed * power);
            bl.setPower(-plusMin * speed * power);
            previousTime = currentTime;
            previousError = currentError;
        }
        setMotorPower(0);
    }

    public void turnPDT(int tarDegree, double speed, double timeout)
    {
        double curDiff;
        double startDiff = tarDegree - getGyroYaw();
        double p = 0;
        curDiff = startDiff;
        double plusMin = 0;
        elapsedTime.reset();
        while(Math.abs(curDiff) > 1 && elapsedTime.milliseconds() < timeout)
        {
            curDiff = tarDegree - getGyroYaw();
            double trueDiff = tarDegree - getGyroYaw();
            int calc = Math.abs((int)(tarDegree - curDiff));
            p = Math.abs(curDiff)/Math.abs(startDiff);
            p = p * Math.signum(trueDiff);
            plusMin = .1 * Math.signum(trueDiff);
            bl.setPower(speed * p + plusMin);
            fl.setPower(speed * p + plusMin);
            br.setPower(speed * p + plusMin);
            fr.setPower(speed * p + plusMin);
            telemetry.addData("getGyrowyaw", getGyroYaw());
            telemetry.addData("curdiff", curDiff);
            telemetry.addData("startd", startDiff);
            telemetry.update();
        }
        bl.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        fr.setPower(0);
    }

    public void clamp(boolean isOpen) {
        if(!isOpen){
            claw.setPosition(1);
        } else {
            claw.setPosition(.5);
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
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetLiftEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int encoderAVG() {
        int avg = Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition());
        return avg/4;
    }

}
