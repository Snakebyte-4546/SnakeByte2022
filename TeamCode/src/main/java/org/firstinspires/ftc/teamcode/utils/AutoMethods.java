package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    public Servo right;
    public Servo left;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


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
        resetLiftEncoder();

        right = auto.hardwareMap.servo.get("rightServo");
        left = auto.hardwareMap.servo.get("leftServo");
        right.setPosition(1);
        left.setPosition(0);
    }

    public void setMotorPower(double power) {
        fl.setPower(-power);
        br.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
    }

    public void moveLift(double speed, int position) {
        if (position == 3) {
            ticks = 500;
        } else if (position == 2) {
            ticks = 400;
        } else if (position == 1) {
            ticks = 300;
        }
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(ticks);
        lift.setPower(speed);
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

    public void clamp(boolean isOpen) {
        if(!isOpen){
            right.setPosition(.75);
            left.setPosition(.25);
        } else {
            right.setPosition(1);
            left.setPosition(0);
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
