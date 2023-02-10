package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp(name = "Modular- v0.2",group = "roadrunner")
public class ModularTeleOp extends LinearOpMode {

    public DcMotorEx lift;
    public BNO055IMU imu;


//  DRIVETRAIN PARAMS
    public static double MAX_SPEED = 1.0;
    public static double SLOW_SPEED_MULTIPLIER = 5.0;

    public double speed = .5;
    public double maxSpeed = 1;
    public Pose2d currentPose;
    public Vector2d targetVector;
    public Vector2d targetVectorError;
    public double xError;
    public double yError;
    public double currentHeading;
    public double targetHeading;
    double targethHeadingError;
    ElapsedTime timeFromVectorChange = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);



//  LIFT PARAMS
    public static double LIFT_SPEED = 150;
    public int liftTarget;
    public int liftMax; //lower center of grav for drivetrain


//  GENERAL PARAMS
    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


//  RIGHT STICK PARAMS
    public static double ANGLE_LOCK_WEIGHT = .5;
    public double angleOf_RStick;
    public double angleOf_RStick_AngleLocked;
    public double angleLockDifference;
    public double angleLockThreshold;

//  TOUCHPAD PARAMS
    public double startingFingerAngle = 0;
    public double currentFingerError = 0;
    public double initialHeading;
    public boolean precisionMode;
    ElapsedTime timeSincePMToggle = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime timeFrom2Fingers = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public void runOpMode() {

//      DRIVETRAIN SETUP
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(-30, 0, Math.toRadians(0)));


//      IMU SETUP
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

//      LIFT SETUP
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//      HEADING TUNER


        waitForStart();
        currentPose = drive.getPoseEstimate();
        targetVector = new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());
        currentHeading = drive.getExternalHeading();



        runtime.reset();
        timeSincePMToggle.reset();
        timeFrom2Fingers.reset();
        timeFromVectorChange.reset();



        while (opModeIsActive() && !isStopRequested()) {


            drive.updatePoseEstimate();

//      CONTROL LOOP

//          heading tuning
            if (gamepad1.left_bumper) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading() - Math.toRadians(speed * 2.2)));
            } else if (gamepad1.right_bumper) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading() + Math.toRadians(speed * 2.2)));
            }



//          HEADING CALC//heading reset
            psButtonCheck_HeadingReset(drive);
            if (headingCheck_2Fingers(drive)) {//nothing
            }
            else if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1 || gamepad1.right_stick_y > .1 || gamepad1.right_stick_y < -.1) {
                targetHeading = rightStick_Heading(drive);
            }

//          VECTOR CALC
            if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1 || gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1) {
                targetVector = leftStick_Vector(drive);
            }
            if (gamepad1.right_stick_button && timeSincePMToggle.time() > 400) {
                timeSincePMToggle.reset();
                precisionMode = !precisionMode;
            }

            vectorErrorCalc(drive);
            headingErrorCalc(drive);
            drive.setWeightedDrivePower(new Pose2d(targetVectorError, targethHeadingError));
            drive.update();
            //telemetry.addData("liftPosition", lift_currentPosition);
        }


    }


//  VECTOR METHODS
    public void vectorErrorCalc(MecanumDrive drive) {
        currentPose = drive.getPoseEstimate();
        xError = targetVector.getX() - drive.getPoseEstimate().getX();
        yError = targetVector.getY() - drive.getPoseEstimate().getY();
        if (xError < 1 && xError > -1) {
            xError = 0;
        }

        if (yError < 1 && yError > -1) {
            yError = yError * Math.abs(yError * .5);
        }
        targetVectorError = new Vector2d(xError, yError).rotated(-drive.getPoseEstimate().getHeading());
    }


    public Vector2d leftStick_Vector(MecanumDrive drive) {
        timeFromVectorChange.reset();
        return new Vector2d((currentPose.getX() - gamepad1.left_stick_y * 2), (currentPose.getY() - gamepad1.left_stick_x * 2));
    }



//  HEADING METHODS
    public double headingErrorCalc(MecanumDrive drive) {
        double actualHeading = Math.toDegrees(drive.getExternalHeading());
        targethHeadingError = targetHeading - actualHeading;
        //gamepad1.rumble((int)headingError);
        if (targethHeadingError > 180) {
            targethHeadingError = targetHeading - 360 - actualHeading;
        } else if (targethHeadingError < -180) {
            targethHeadingError = targetHeading + 360 - actualHeading;
        }

        maxSpeed = (45 - targethHeadingError) / 45;
        if (maxSpeed < 0) maxSpeed = 0;
        return Math.toRadians(targethHeadingError);
    }

    private double rightStick_Heading(MecanumDrive drive) {
        angleOf_RStick = Math.toDegrees(Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y)) + 180;
        angleOf_RStick_AngleLocked = angleOf_RStick;
        if (angleOf_RStick < 45) angleOf_RStick_AngleLocked = 0;
        else if ( angleOf_RStick > 315) angleOf_RStick_AngleLocked = 360;
        else if (angleOf_RStick > 45 && angleOf_RStick < 135)  angleOf_RStick_AngleLocked = 90;
        else if (angleOf_RStick > 135 && angleOf_RStick < 225) angleOf_RStick_AngleLocked = 180;
        else if (angleOf_RStick > 225 && angleOf_RStick < 315) angleOf_RStick_AngleLocked = 270;

        angleLockDifference = angleOf_RStick - angleOf_RStick_AngleLocked;
        if (precisionMode) {
            return angleOf_RStick;
        }
        return angleOf_RStick_AngleLocked;
    }

    private void psButtonCheck_HeadingReset(MecanumDrive drive) {
        if (gamepad1.ps) {
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0));
        }
    }

    private boolean HeadingCheck_2Fingers(MecanumDrive drive) {
        if (gamepad1.touchpad_finger_2_x != 0 || gamepad1.touchpad_finger_2_y != 0) {
            if (timeFrom2Fingers.time() > 100) {
                timeFrom2Fingers.reset();
                initialHeading = drive.getExternalHeading();
                startingFingerAngle = (gamepad1.touchpad_finger_2_y - gamepad1.touchpad_finger_1_y) / (gamepad1.touchpad_finger_2_x - gamepad1.touchpad_finger_1_x);
            }
            else {
                timeFrom2Fingers.reset();
                currentFingerError = (gamepad1.touchpad_finger_2_y - gamepad1.touchpad_finger_1_y) / (gamepad1.touchpad_finger_2_x - gamepad1.touchpad_finger_1_x) - startingFingerAngle;
            }
        }
        else
        targetHeading = initialHeading + currentFingerError;
    }
}