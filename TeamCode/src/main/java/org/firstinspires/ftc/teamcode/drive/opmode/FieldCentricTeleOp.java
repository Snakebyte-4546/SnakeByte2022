package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Config
@TeleOp(group = "roadrunner")
public class FieldCentricTeleOp extends LinearOpMode {


    public static double speed;
    public static double LIFTSPEED = 150;
    public static int FOURBARSPEED = 100;

    public DcMotorEx lift;
    public DcMotor fourbar;
    public Servo claw;
    public BNO055IMU imu;

    public double clawClose = 0;


    Vector2d currentVector;
    double headingError;
    double targetHeading;

    //no
    double intendedHeading; //calculated from toutcchpad
    double intendedHeadingError;
    Pose2d pose;
    double angle; //the angle variable the robot lock onto after the touchpad had been released after button press

    //used to lock the heading after timeout ms from last right stick input
    ElapsedTime timeFromLastInput = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //redude if it takes to long to settle, increase if it oscillates backwards
    int inputTimeout = 200;

    //   these two vars have to do with the touchpad, they measure how long you have to press down before
    //  it unlocks the accurate heading control
    //  the time till hold should be set just above how long a normal press takes
    ElapsedTime timeSincePress = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int TimeTillHold = 500;
    boolean alreadyRegistered = false;

    //lift & fourbar pid
    int fourbar_targetPosition = 0;

    ElapsedTime clawToggleCooldown;

    //for heading calculations
    boolean FieldcentricMode = true;


    static double lift_targetPosition = 0;
    double lift_errorTolerance = 10;
    double lift_deltaError;
    double lift_lastError = 0;
    double lift_integral = 0;
    ElapsedTime lift_PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static PIDCoefficients lift_PIDCOEFFS = new PIDCoefficients(8, .002, 5);
    private final PIDCoefficients lift_pidGains = new PIDCoefficients(0, 0, 0);

    static double fbar_targetPosition = 0;
    double fbar_errorTolerance = 30;
    double fbar_deltaError;
    double fbar_integral = 0;
    double fbar_lastError = 0;
    ElapsedTime fbarHoldLow = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    boolean pressFBar;
    public static PIDCoefficients fbar_PIDCOEFFS = new PIDCoefficients(5, 0, 3);
    private final PIDCoefficients fbar_pidGains = new PIDCoefficients(0, 0, 0);
    ElapsedTime clawToogleCooldown = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    //camera stuff
    //OpenCvWebcam camera;
    //boolean isSetup = false;


    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // if the pose is gotten from the auto(poseStorage), or pre
        //drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setPoseEstimate(new Pose2d(-30, 0, Math.toRadians(0)));


        claw = hardwareMap.servo.get("claw");

        timeSincePress.reset();

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);

        fourbar = hardwareMap.dcMotor.get("4bar");
        fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourbar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        claw.setPosition(clawClose);

        speed = .5;
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        targetHeading = drive.getPoseEstimate().getHeading();


        while (opModeIsActive() && !isStopRequested()) {


            // Claw control Driver 2
            if (gamepad2.right_bumper) {
                if (clawToogleCooldown.time() > 500){
                    if (claw.getPosition() == 0) claw.setPosition(1);
                    else claw.setPosition(0);
                    clawToogleCooldown.reset();
                }
            }

            // Heading reset driver 1
            if (gamepad1.ps || gamepad1.right_stick_button) {
                drive.setPoseEstimate(new Pose2d(-30, 0, Math.toRadians(0)));
            }

            // Robot Rest Driver 2
            if (gamepad2.left_bumper) {
                gamepad2.rumbleBlips(1);
                claw.setPosition(0);
                lift_targetPosition = 0;
                PIDLift();
                fourbar.setTargetPosition(0);
                fourbar.setPower(1);
            }

            // Drivetrain speed calculations
            if (gamepad1.right_trigger > 0.1) {
                if (speed > .5) speed -= .02;          //half speed
                if (speed < .5) speed += .02;

            } else if(gamepad1.left_trigger > 0.1){       //quarter speed
                if (speed > .28) speed -= .02;
                else if (speed < .28) speed += .02;

            } else {                                      //full speed
                if (speed < .7) speed += .04;
                if (speed < 1) speed -= .02;
                else if (speed > 1) speed += .02;}
            telemetry.addData("speed ", speed);


            // make drivetrain inputs non linear, not used
            double ProcessedTranslationY = -gamepad1.left_stick_y;
            double ProcessedTranslationX = -gamepad1.left_stick_x;


            // Movement calc Driver 1

            currentVector = new Vector2d(0, 0).rotated(-drive.getPoseEstimate().getHeading());

            if (gamepad1.dpad_up) {
                currentVector = new Vector2d(speed,
                        0).rotated(-drive.getPoseEstimate().getHeading());
                gamepad1.rumble(200);
            } else if (gamepad1.dpad_down) {
                currentVector = new Vector2d(-speed,
                        0).rotated(-drive.getPoseEstimate().getHeading());
                gamepad1.rumble(200);
            } else if (gamepad1.dpad_left) {
                currentVector = new Vector2d(0,
                        speed).rotated(-drive.getPoseEstimate().getHeading());
                gamepad1.rumble(200);
            } else if (gamepad1.dpad_right) {
                currentVector = new Vector2d(0,
                        -speed).rotated(-drive.getPoseEstimate().getHeading());
                gamepad1.rumble(200);
            } else if (gamepad1.right_trigger > .3) {
                currentVector = new Vector2d(-gamepad1.left_stick_y * speed,
                        -gamepad1.left_stick_x * speed).rotated(-drive.getPoseEstimate().getHeading());
            } else {
                currentVector = new Vector2d(-gamepad1.left_stick_y * speed,
                        -gamepad1.left_stick_x * speed).rotated(-drive.getPoseEstimate().getHeading());
            }
            gamepad1.rumble((int)(Math.abs(gamepad1.left_stick_x * (speed * 100) + Math.abs(gamepad1.left_stick_y * (speed * 100)))));



            //Heading calc Driver 1
            if (gamepad1.left_bumper) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading() - Math.toRadians(2)));
            }
            else if (gamepad1.right_bumper) {
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading() + Math.toRadians(2)));
            }

            // Driver 1 heading snap, fallback to stick control
                if (gamepad1.triangle) angle = 0;
                else if (gamepad1.square) angle = 90;
                else if (gamepad1.cross) angle = 180;
                else if (gamepad1.circle) angle = 270;
                else if (gamepad1.right_stick_x >.05 || gamepad1.right_stick_x < -.05 || gamepad1.right_stick_y >.05 || gamepad1.right_stick_y < -.05) {
                    angle = Math.toDegrees(Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y)) + 180;
                    timeFromLastInput.reset();
                    } /*else if (timeFromLastInput.time() > 500) {
                    angle = Math.toDegrees(drive.getExternalHeading());}*/
            targetHeading = angle;
                pose = new Pose2d(currentVector, HeadingCalculator_Angle(drive, angle));
            drive.setWeightedDrivePower(
                    pose);
            drive.update();


            //lif pos set Driver 2
            if (gamepad2.dpad_up && lift_targetPosition < 2300) {          //manual up
                gamepad2.rumble(2);
                lift_targetPosition += LIFTSPEED;
            }
            if (gamepad2.dpad_down && lift_targetPosition > -100) {
                gamepad2.rumble(2);//manual down
                lift_targetPosition += -LIFTSPEED;
            }
            if (gamepad2.dpad_right) {
                gamepad2.rumbleBlips(1);
                lift_targetPosition = 2500;
            }
            if (gamepad2.dpad_left) {          //lift down
                gamepad2.rumbleBlips(1);
                lift_targetPosition = 0;
            }
            PIDLift();


            //four bar manual for driver 2
            if (gamepad2.right_stick_y > .04 || gamepad2.right_stick_y < -.04) {
                claw.setPosition(fourbar.getCurrentPosition() + gamepad2.right_stick_y * 2);
                fourbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }   else if (gamepad2.a) {  //four bar pos
                claw.setPosition(0);
                if (pressFBar) {
                    fourbar.setTargetPosition(-300);
                    fourbar.setPower(17889);
                    fbarHoldLow.reset();
                    pressFBar = false;
                } else if (fbarHoldLow.time() < 500) {
                } else if (fbarHoldLow.time() > 500) {
                    fourbar_targetPosition = fourbar_targetPosition - 20;
                }
                //fbar_targetPosition = 40;
            } else if (!pressFBar) {
                pressFBar = true;
                fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } else if (gamepad2.b && fourbar.getCurrentPosition() > 0) {
                claw.setPosition(0);//fourbar front
                fourbar_targetPosition = fourbar.getCurrentPosition() - FOURBARSPEED;
                //fbar_targetPosition = 350;
            } else if (gamepad2.y) {//fourbar up
                claw.setPosition(0);
                fourbar_targetPosition = 1000;
            } else if (gamepad2.x && fourbar.getCurrentPosition() < 960) {
                claw.setPosition(0);//fourbar back
                fourbar_targetPosition = fourbar.getCurrentPosition() + FOURBARSPEED;
                //fbar_targetPosition = 910;
            }
            fourbar.setTargetPosition(fourbar_targetPosition);
            fourbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fourbar.setPower(1);
            //PIDfBar();


            Pose2d currentPose = drive.getPoseEstimate();


            //from touch input to radial, robot matching numbers
            double angleOfTouchpad = (360 - (Math.toDegrees(Math.atan2(gamepad1.touchpad_finger_1_x * 2, gamepad1.touchpad_finger_1_y)))) - 360;
            if (angleOfTouchpad < 0) angleOfTouchpad += 360;


            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading", currentPose.getHeading());
            telemetry.addData("liftTarget", lift_targetPosition);
            telemetry.addData("fourBarTarget", fbar_targetPosition);
            telemetry.addData("heading target", targetHeading);
            telemetry.addData("angle of right stick", Math.toDegrees(Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y)) + 180);
            telemetry.addData("angle of touchPad: ", angleOfTouchpad);
            telemetry.update();


        }

    }

    public double HeadingCalculator_InputLock(MecanumDrive drive) {
        double actualHeading = Math.toDegrees(drive.getExternalHeading());
        if (gamepad1.right_stick_x >.05 || gamepad1.right_stick_x < -.05 || gamepad1.right_stick_y >.05 || gamepad1.right_stick_y < -.05) {
            timeFromLastInput.reset();
            targetHeading = Math.toDegrees(Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y)) + 180;
        }
        else if (timeFromLastInput.time() < inputTimeout) {
            //gamepad1.rumble(2);
            targetHeading = actualHeading;
        }
        else {
            headingError = targetHeading - actualHeading;
            //gamepad1.rumble((int)headingError * 4);

        }
        if (headingError > 180) headingError = targetHeading-360 -  actualHeading;
        else if (headingError <  -180) headingError = targetHeading+360- actualHeading;

        return Math.toRadians(headingError);
    }

    public double HeadingCalculator_FieldCentric(MecanumDrive drive) {
        double actualHeading = Math.toDegrees(drive.getExternalHeading());
        double targetHeading = 0;
        if (gamepad1.right_stick_x >.05) targetHeading =  Math.toDegrees(Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y)) + 180;
        //telemetry.addData("FIELDCENTRIC_HEADING", targetHeading);
        //telemetry.update();
        headingError = targetHeading - actualHeading;

        if (headingError > 180) headingError = headingError - 360;
        else if (headingError <  -180) headingError = headingError + 360;

        return Math.toRadians(headingError);
    }

    public void fourbarFindRest(MecanumDrive drive) {
       fourbar.setTargetPosition(-2000);
       fourbar.setPower(.8);
       sleep(500);
       fourbar.setPower(0);
       fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       fourbar.setTargetPosition(40);
       fourbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public double HeadingCalculator_Angle(MecanumDrive drive, double targetHeading) {
        double actualHeading = Math.toDegrees(drive.getExternalHeading());
        headingError = targetHeading -  actualHeading;
        //gamepad1.rumble((int)headingError);
        if (headingError > 180) {headingError = targetHeading-360 -  actualHeading;}
        else if (headingError <  -180) {headingError = targetHeading+360- actualHeading;}
        return Math.toRadians(headingError);
    }


    public boolean liftAtTarget() {
        return (lift.getCurrentPosition() >= (lift_targetPosition - lift_errorTolerance) && lift.getCurrentPosition() <= (lift_targetPosition + lift_errorTolerance)) && !(lift_deltaError >= 25);
    }
    public void PIDLift() {
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
        return (fourbar.getCurrentPosition() >= (fbar_targetPosition - fbar_errorTolerance) && fourbar.getCurrentPosition() <= (fbar_targetPosition + fbar_errorTolerance)) && !(fbar_deltaError >= 25);
    }
}


