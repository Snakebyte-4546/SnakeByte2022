package org.firstinspires.ftc.teamcode.opmodes.roadrunnerTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@TeleOp(group = "drive")
public class CustomPIDTuner extends LinearOpMode {
    public static double POSITION = 0;
    public static double GRAVITY = .1;
    public static double MAXTICKS = 1;
    public static boolean LIFT = false;
    public static boolean FOURBAR = false;

    public static PIDCoefficients PIDCOEFFS = new PIDCoefficients(1, 0, 0);

    double lastError = 0;
    double integral = 0;
    private final PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    //AutoMethods robot = new AutoMethods();
    DcMotorEx motor;
    DcMotorEx motor2;
    DcMotorEx motor3;
    DcMotorEx motor4;
    private Servo claw;



    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        //robot.ready(this);


        //liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //robot.claw(false);
        //robot.moveLift(1500);
        //motor = hardwareMap.get(DcMotorEx.class, "4bar");
        //motor = hardwareMap.get(DcMotorEx.class, "lift");
        motor = hardwareMap.get(DcMotorEx.class, "4bar");
        if (LIFT) {motor = hardwareMap.get(DcMotorEx.class, "lift");}
        /*else {motor = hardwareMap.get(DcMotorEx.class, "fl");
              motor2 = hardwareMap.get(DcMotorEx.class, "fr");
              motor2.setDirection(DcMotorSimple.Direction.REVERSE);
              motor3 = hardwareMap.get(DcMotorEx.class, "bl");
              motor3.setDirection(DcMotorSimple.Direction.REVERSE);
              motor4 = hardwareMap.get(DcMotorEx.class, "br");}*/



        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("move lift to  " + POSITION + " position");
        telemetry.addLine("gravitational force " + MAXTICKS + " applied");
        telemetry.addLine("lift enabled: " + LIFT+ " status");
        telemetry.addLine("4bar enabled: " + FOURBAR + " status");
        telemetry.addData("gravitational force ", MAXTICKS);
        telemetry.addData("error ", lastError);
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();
        telemetry.update();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
                telemetry.addLine("Encoder Position:  " + motor.getCurrentPosition());
                telemetry.addLine("Target Position:  " + POSITION);
                telemetry.addLine("Target Position:  " + MAXTICKS);
                telemetry.addLine("lift enabled: " + LIFT);
                telemetry.addLine("4bar enabled: " + FOURBAR);
                telemetry.addData("integral ", integral);
                telemetry.addData("Error ", lastError);
                telemetry.addData("Target Position ", POSITION * 2230);
                telemetry.addData("Actual Position ", motor.getCurrentPosition());
                telemetry.update();
                PID(POSITION * 950 * MAXTICKS);
            }
        }
    }

    public void PID(double targetPosition) {
        PIDTimer.reset();

        double currentPosition = motor.getCurrentPosition();
        double error = targetPosition - currentPosition;
        telemetry.addLine("Error:  " + error);
        telemetry.addData("Error", error);

        double deltaError = error - lastError;
        double derivative = deltaError / PIDTimer.time();

        integral += error * PIDTimer.time();

        pidGains.kP = error * PIDCOEFFS.kP;
        pidGains.kI = integral * PIDCOEFFS.kI;
        pidGains.kD = derivative * PIDCOEFFS.kD;

        motor.setVelocity(pidGains.kP + pidGains.kI + pidGains.kD);
        lastError = error;
    }
}
