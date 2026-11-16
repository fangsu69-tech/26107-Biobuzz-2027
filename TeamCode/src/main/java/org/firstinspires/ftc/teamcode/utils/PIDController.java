package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double Kp;
    private double Ki;
    private double Kd;

    private double lastError = 0;
    private double integral = 0;
    private boolean angleWrap = false;

    private double integralMax = 1.0;      // anti-windup clamp
    private double outputMin = -1.0;       // output clamp min
    private double outputMax = 1.0;        // output clamp max

    private final ElapsedTime timer = new ElapsedTime();

    /** Constructor for general PID (no angle wrapping) */
    public PIDController(double Kp, double Ki, double Kd) {
        this(Kp, Ki, Kd, false);
    }

    /** Constructor with optional angle wrapping */
    public PIDController(double Kp, double Ki, double Kd, boolean angleWrap) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.angleWrap = angleWrap;
        reset();
    }

    /** Calculate PID output given reference and current state */
    public double output(double reference, double state) {
        double error = angleWrap ? wrapAngle(reference - state) : reference - state;
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 1e-6;
        // Integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integralMax, integralMax);
        // Derivative
        double derivative = (error - lastError) / dt;
        // PID output
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        lastError = error;
        // Clamp output
        return clamp(output, outputMin, outputMax);
    }


    /** * Calculate PID output given a pre-calculated error.
     * This is useful when the sensor (like Limelight tx) provides the error directly.
     * NOTE: This assumes the input 'error' is in the units expected by your gains (e.g., degrees).
     */
    public double calculate(double error) {
        // If angle wrapping was enabled, we need to wrap the error here
        // (though for Limelight tx, angleWrap is usually false)
        if (angleWrap) {
            error = wrapAngle(error);
        }

        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 1e-6; // Prevent division by zero

        // Integral with anti-windup (same as in output method)
        integral += error * dt;
        integral = clamp(integral, -integralMax, integralMax);

        // Derivative (same as in output method)
        double derivative = (error - lastError) / dt;

        // PID output
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        lastError = error;

        // Clamp output
        return clamp(output, outputMin, outputMax);
    }

    /** Reset integral, derivative history */
    public void reset() {
        integral = 0;
        lastError = 0;
        timer.reset();
    }

    /** Helper: clamp value between min and max */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /** Wrap angle to [-π, π] */
    private double wrapAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
    /** Update PID gains */
    public void setGains(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /** Configure integral clamp */
    public void setIntegralMax(double max) {
        this.integralMax = max;
    }

    /** Configure output clamp symmetrically */
    public void setOutputMax(double max) {
        this.outputMax = max;
        this.outputMin = -max;
    }

    /** Configure output clamp asymmetrically */
    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }
}
