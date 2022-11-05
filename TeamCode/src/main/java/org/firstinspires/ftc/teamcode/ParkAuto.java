package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.AutoMethods;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name = "Park Auto", group = "Auto")
public class ParkAuto extends LinearOpMode {
    AutoMethods autoMethods = new AutoMethods();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int tagOfInterest = 0;

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;


    @Override
    public void runOpMode() throws InterruptedException {
        // Camera Setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        // Setup Camera Pipeline
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error:", errorCode);
                telemetry.update();
            }
        });

        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
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
        while (!isStarted() && !isStopRequested()) {
            // Update with latest data
            ArrayList < AprilTagDetection > currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            // Check if tags are detected
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                // Ignoring all other tags that may be present
                for (AprilTagDetection tag: currentDetections) {
                    if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                        tagOfInterest = tag.id;
                        tagFound = true;
                        break;
                    }
                }
                // If a tag is found
                if (tagFound) { telemetry.addData("Tag Found:", tagOfInterest); }

                // Something is wrong
            } else {
                telemetry.addLine("Tag not currently found:");

                if (tagOfInterest == 0) {
                    telemetry.addLine("(no tags have been seen)");
                } else {
                    telemetry.addLine("\nBut the tag has been seen before,");
                }
            }
            telemetry.update();
        }

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            if(tagOfInterest == 1){
                MoveInchEncoder(-.25,650);
                Strafe(.25,true, 650);
            } else if(tagOfInterest == 2){
                Strafe(.25,true, 650);
            } else {
                MoveInchEncoder(.25,650);
                Strafe(.25,true, 650);
            }
        }
    }
    public void setMotorPower(double power){
        fl.setPower(power);
        br.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
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

