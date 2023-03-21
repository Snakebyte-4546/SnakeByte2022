package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp(name = "Modular- v0.2",group = "roadrunner")
public class ModularTeleOp extends LinearOpMode {

    //public DcMotorEx lift;
    public BNO055IMU imu;


    //  DRIVETRAIN PARAMS
    public static double MAX_SPEED = 1.0;
    public static double SLOW_SPEED_MULTIPLIER = 5.0;

    public double speed = .5;
    public double maxSpeed = 1;


    //  VECTOR PARAMS
    public Pose2d currentPose;
    public Vector2d targetVector;
    public Vector2d targetVectorError = new Vector2d(0,0);
    public double xError;
    public double yError;
    public  boolean precisionVector = true;

    double angleOf_LStick;

    int timeAtLastPVChange;
    ElapsedTime timeFromVectorChange = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int vectorInputTimeout = 300;


    //  HEADING PARAMS
    public double currentHeading;
    public double targetHeading;
    public double targetHeadingError = 0;
    public boolean precisionHeading = true;
    int timeAtLastPHChange = 0;


    //  LIFT PARAMS
    public static double LIFT_SPEED = 150;
    public int liftTarget;
    public int liftMax; //lower center of grav for drivetrain




    //  RIGHT STICK PARAMS
    public static double ANGLE_LOCK_WEIGHT = .5;
    public double angleOf_RStick;
    public double angleOf_RStick_AngleLocked;
    public double angleLockThreshold;

    //  TOUCHPAD PARAMS
    public double startingFingerAngle = 0;
    public double currentFingerError = 0;
    public double initialHeading;
    ElapsedTime timeFrom2Fingers = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    public void runOpMode() {

//      DRIVETRAIN SETUP
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));



//      IMU SETUP
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

//      LIFT SETUP
        //lift = hardwareMap.get(DcMotorEx.class, "lift");
        //lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//      HEADING TUNER
        boolean adjustingHeading;
        ElapsedTime timeSinceHeadingAdjust = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        waitForStart();
        currentPose = drive.getPoseEstimate();
        targetVector = new Vector2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY());
        currentHeading = drive.getExternalHeading();



        runtime.reset();
        timeFrom2Fingers.reset();
        timeFromVectorChange.reset();



        while (opModeIsActive() && !isStopRequested()) {


            drive.updatePoseEstimate();
            currentHeading = Math.toDegrees(drive.getPoseEstimate().getHeading());
            currentPose = drive.getPoseEstimate();




//      CONTROL LOOP \/ \/ \/  CONTROL LOOP

//          heading tuning
            headingReset_PSButton(drive);
            if (gamepad1.left_bumper) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading() - Math.toRadians(1.5 + timeSinceHeadingAdjust.time() / 1000)));
            }
            else if (gamepad1.right_bumper) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading() + Math.toRadians(1.5 + timeSinceHeadingAdjust.time() / 1000)));
            }
            else {timeSinceHeadingAdjust.reset();}

//          INPUT CHECKS
            precisionToggleCheck();
            speedCalculator();


//          HEADING CALC
            //if (headingReset_PSButton(drive))       {}
            //else if (headingCheck_2Fingers(drive)) {}
            if (gamepad1.right_trigger > .3)    {headingCheck_RStickBasic(drive);}
            else if (headingCheck_RStick(drive))    {}


//          VECTOR CALC
            if (vectorCheckAndSet_LStick_NoOdom(drive))         {}

            //vectorErrorCalc(drive);
            //vectorErrorCalc_MoreRawValues(drive);
            headingErrorCalc(drive);
            telemetry.update();
            drive.setWeightedDrivePower(new Pose2d(targetVectorError, targetHeadingError));
            drive.update();
            //telemetry.addData("liftPosition", lift_currentPosition);
        }


    }


    //  VECTOR METHODS
    public void vectorErrorCalc(MecanumDrive drive) {
        if (timeFromVectorChange.time() > vectorInputTimeout){
            targetVectorError = new Vector2d(0,0);
        }
        else {
            xError = targetVector.getX() - currentPose.getX();
            yError = targetVector.getY() - currentPose.getY();
            if (xError < 1 && xError > -1) {
                xError = 0;
            }

            if (yError < 1 && yError > -1) {
                yError = yError * Math.abs(yError * .5);
            }
            targetVectorError = new Vector2d(xError, yError).rotated(-currentPose.getHeading());
        }
    }

    public void vectorErrorCalc_MoreRawValues(MecanumDrive drive) {
        if (timeFromVectorChange.time() > vectorInputTimeout / 5){
            targetVectorError = new Vector2d(0,0);
        }
        else {
            xError = targetVector.getX() - currentPose.getX();
            yError = targetVector.getY() - currentPose.getY();

            targetVectorError = new Vector2d(xError, yError).rotated(-currentPose.getHeading());
        }
    }

    public boolean vectorCheckAndSet_LStick_NoOdom(MecanumDrive drive) {
        if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1 || gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1) {
            timeFromVectorChange.reset();
            if (precisionVector) {
                targetVectorError = new Vector2d(-gamepad1.left_stick_y * speed, -gamepad1.left_stick_x * speed).rotated(-drive.getPoseEstimate().getHeading());
            }
            else {
                angleOf_LStick = Math.toDegrees(Math.atan2(gamepad1.left_stick_x, gamepad1.left_stick_y)) + 180;
                if (angleOf_LStick < 45  || angleOf_LStick > 315) {targetVectorError = new Vector2d(-gamepad1.left_stick_y * speed, 0);}
                else if (angleOf_LStick > 45 && angleOf_LStick < 135) {targetVectorError = new Vector2d(0, -gamepad1.left_stick_x * speed);}
                else if (angleOf_LStick > 135 && angleOf_LStick < 225) {targetVectorError = new Vector2d(-gamepad1.left_stick_y * speed, 0);}
                else if (angleOf_LStick > 225 && angleOf_LStick < 315) {targetVectorError = new Vector2d(0, -gamepad1.left_stick_x * speed);}
            }
            return true;
        }
        targetVectorError = new Vector2d(0,0);
        return false;
    }

    public boolean vectorCheck_LStick(MecanumDrive drive) {
        if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1 || gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1) {
            timeFromVectorChange.reset();
            targetVector = new Vector2d((currentPose.getX() - gamepad1.left_stick_y * 2), (currentPose.getY() - gamepad1.left_stick_x * 2));
            return true;
        }
        return false;
    }



    //  HEADING METHODS
    public void headingErrorCalc(MecanumDrive drive) {
        telemetry.addLine("current IMU heading: " + currentHeading);
        telemetry.addLine("target heading: " + targetHeading);
        targetHeadingError = targetHeading - currentHeading;
        if (targetHeadingError > 180) {targetHeadingError = targetHeading - 360 - currentHeading;
        } else if (targetHeadingError < -180) {targetHeadingError = targetHeading + 360 - currentHeading;
        }
        targetHeadingError = Math.toRadians(targetHeadingError);
    }

    private boolean headingCheck_RStick(MecanumDrive drive) {
        if (gamepad1.right_stick_x > .3 || gamepad1.right_stick_x < -.3 || gamepad1.right_stick_y > .3 || gamepad1.right_stick_y < -.3) {
            angleOf_RStick = Math.toDegrees(Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y)) + 180;

            if (precisionHeading) {
                targetHeading = angleOf_RStick;
            }
            else {
                angleOf_RStick_AngleLocked = angleOf_RStick;
                if (angleOf_RStick < 45) {angleOf_RStick_AngleLocked = 0;}
                else if (angleOf_RStick > 315) {angleOf_RStick_AngleLocked = 360;}
                else if (angleOf_RStick > 45 && angleOf_RStick < 135) {angleOf_RStick_AngleLocked = 90;}
                else if (angleOf_RStick > 135 && angleOf_RStick < 225) {angleOf_RStick_AngleLocked = 180;}
                else if (angleOf_RStick > 225 && angleOf_RStick < 315) {angleOf_RStick_AngleLocked = 270;}
                targetHeading = angleOf_RStick_AngleLocked;}
            return true;
        }
        return false;
    }

    private boolean headingCheck_RStickBasic(MecanumDrive drive) {
        if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
            targetHeading = currentHeading - gamepad1.right_stick_x * 40;
            return true;
        }
        return false;
    }



    private boolean headingCheck_2Fingers(MecanumDrive drive) {
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

            targetHeading = initialHeading + currentFingerError;
            return true;
        }
        else return false;
    }

    private boolean headingReset_PSButton(MecanumDrive drive) {
        if (gamepad1.ps) {
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), 0));
            return true;
        }
        return false;
    }

    private void speedCalculator() {
        speed = .3 + (gamepad1.left_trigger * .7);
    }

    private void precisionToggleCheck() {
        if (gamepad1.left_stick_button && (runtime.time() - timeAtLastPVChange) > 400) {
            timeAtLastPVChange = (int)runtime.time();
            precisionVector = !precisionVector;
        }
        if (gamepad1.right_stick_button && (runtime.time() - timeAtLastPHChange) > 400) {
            timeAtLastPHChange = (int)runtime.time();
            precisionHeading = !precisionHeading;
        }
    }
}