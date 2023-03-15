package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

public class AutoMethods {
    public DcMotor lift;
    public Servo claw;

    int ticks = 0;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public void ready(LinearOpMode auto) {
        lift = auto.hardwareMap.dcMotor.get("lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetLiftEncoder();
        claw = auto.hardwareMap.servo.get("claw");
        clamp(false);
    }

    public int getHeight(String code){
        int returnHeight = 0;
        if(code.equals("high")){
            returnHeight = 4100;
        }
        else if(code.equals("mid")){
            returnHeight = 2300;
        }
        else if(code.equals("low")){
            returnHeight = 1800;
        }
        else if(code.equals("driving")){
            returnHeight = 0;
        }
        else if(code.equals("s5")){
            returnHeight = 1250;
        } else if(code.equals("s4")){
            returnHeight = 1075;
        } else if(code.equals("s3")){
            returnHeight = 900;
        } else if(code.equals("s2")){
            returnHeight=825;
        } else if(code.equals("s1")){
            returnHeight=750;
        }
        return returnHeight;
    }

    public void moveLift(double speed, String position) {
        ticks = getHeight(position);
        lift.setTargetPosition(ticks);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(speed);
    }

    public void clamp(boolean isOpen) {
        if(!isOpen){
            claw.setPosition(.5);
        } else {
            claw.setPosition(1);
        }
    }

    public void resetLiftEncoder() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

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
        FtcDashboard.getInstance().startCameraStream(camera, 15);
        return aprilTagDetectionPipeline;
    }

    public ArrayList<AprilTagDetection> getTag(AprilTagDetectionPipeline pipeline) {
        return pipeline.getLatestDetections();
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
        }
    }

    public void scoreHigh() {
        moveLift(1, "high");
        wait(5000);
        /*
        wait(3000);
        clamp(true);
        wait(1000);
        moveLift(1, "driving");
         */
    }

    public void pickStack(int i){
        clamp(true);
        if(i == 1){
            moveLift(1, "s5");
        } else if(i == 2){
            moveLift(1, "s4");
        } else if(i == 3){
            moveLift(1, "s3");
        }
        wait(500);
        clamp(false);
    }
}