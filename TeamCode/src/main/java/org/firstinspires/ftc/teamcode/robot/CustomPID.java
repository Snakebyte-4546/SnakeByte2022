package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


public class CustomPID {
    public double POSITION = 0;

    public PIDCoefficients PIDCOEFFS = new PIDCoefficients(1, 0, 0);

    double lastError = 0;
    double integral = 0;
    private final PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    private String name;
    double encoderPosition;
    boolean HD = true;

    double error;
    double targetPosition;
    double derivative;
    double deltaError;
    double result;
    int errorTolerance;
    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public void motorSetup( int errorTolerance, PIDCoefficients PIDCOEFFS) {
        this.PIDCOEFFS = PIDCOEFFS;
        this.errorTolerance = errorTolerance;
    }

    public void setTarget(double target) {
        targetPosition = target;
        //PIDTimer.reset();
    }
    public boolean atTarget() {
        return (encoderPosition >= (targetPosition - errorTolerance) && encoderPosition <= (targetPosition +errorTolerance)) && !(deltaError >= 25);
    }

    public double getError() {
        return error;
        //PIDTimer.reset();
    }

            //telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            //telemetry.addLine("Encoder Position:  " + motor.getCurrentPosition());
            //telemetry.addLine("Target Position:  " + POSITION);
            //telemetry.update();
            //PID(POSITION);

    public double calculate(double position) {
        encoderPosition = position;
        PIDTimer.reset();
        error = targetPosition - encoderPosition;

        deltaError = error - lastError;
        derivative = deltaError / PIDTimer.time();

        integral += error * PIDTimer.time();

        pidGains.kP = error * PIDCOEFFS.kP;
        pidGains.kI = integral * PIDCOEFFS.kI;
        pidGains.kD = derivative * PIDCOEFFS.kD;

        lastError = error;

        return pidGains.kP + pidGains.kI + pidGains.kD;
    }
}
