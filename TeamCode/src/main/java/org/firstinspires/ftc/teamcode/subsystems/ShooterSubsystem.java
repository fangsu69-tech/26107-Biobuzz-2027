package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;
@Configurable
public class ShooterSubsystem {
    private DcMotorEx shooterMotor;
    private PIDController velocityPID;


    // Constants for conversion
    private final double TICKS_PER_ROTATION = 28;
    private double targetTicksPerSec = 0;
    private long stateTransitionTime = 0;
    private boolean lastReadyState = false;

    @Sorter(sort = 0) static double kP = 0.005, kI = 0.01, kD = 0.0002, kF = 0.0005;
    @Sorter(sort = 1) static double RPM_TOLERANCE = 200;


    public void initialize(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        shooterMotor = hwMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        velocityPID = new PIDController(kP, kI, kD);
        velocityPID.setOutputMax(1.0);
    }

    /**
     * Converts desired RPM to Ticks per Second and sets the target.
     * @param rpm Desired rotations per minute
     */
    public void setRPM(double rpm) {
        this.targetTicksPerSec = (rpm * TICKS_PER_ROTATION) / 60.0;
        TelemetryUtils.addData("targetRPM", rpm);
    }

    /**
     * Updates the PID loop. Call this every loop.
     */
    public void run() {
        double currentTicksPerSec = shooterMotor.getVelocity(); // ticks per second

        double feedforward = targetTicksPerSec * kF;
        double pidCorrection = velocityPID.output(targetTicksPerSec, currentTicksPerSec);

        double totalPower = feedforward + pidCorrection;

        shooterMotor.setPower(totalPower);
    }

    /**
     * Returns true if the shooter is within the allowed RPM tolerance.
     * probably going to be used for lights/rumble when human input accounts for small delay anyways
     */
    public boolean isReadyToFire() {
        // If the target is 0, we aren't "ready" to fire
        if (targetTicksPerSec == 0) return false;

        double currentRPM = getCurrentRPM();
        double targetRPM = (targetTicksPerSec * 60.0) / TICKS_PER_ROTATION;

        return Math.abs(currentRPM - targetRPM) < RPM_TOLERANCE;
    }


    // will probably be used for autos and the such when enuring a good shot is necessary
    public boolean isStableReadyToFire() {
        boolean currentlyReady = isReadyToFire();

        // If state changed, reset the timer
        if (currentlyReady != lastReadyState) {
            stateTransitionTime = System.currentTimeMillis();
            lastReadyState = currentlyReady;
        }

        // Only return true if it's been ready for more than 100ms
        return currentlyReady && (System.currentTimeMillis() - stateTransitionTime > 100);
    }


    public void stop() {
        setRPM(0);
        shooterMotor.setPower(0);
        velocityPID.reset(); // Clear integral sum when stopping
        TelemetryUtils.addData("targetRPM", 0);
    }

    // Helper to see how close you are to your target in Telemetry
    public double getCurrentRPM() {
        return (shooterMotor.getVelocity() * 60.0) / TICKS_PER_ROTATION;
    }

}
