package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class DriveSubsystem {

    // Hardware Variables
    private DcMotorEx fl, fr, bl, br;
    private GoBildaPinpointDriver odo;
    private Limelight3A limelight;
    private VoltageSensor batteryVoltageSensor;
    private PIDController headingPID;
    private final ElapsedTime visionTimer = new ElapsedTime();
    private double lastKnownTx = 0;
    private double lastKnownTy = 0;

    private double targetHeading = 0;
    private boolean isLocking = false;
    public boolean acceleration = false;

    public double goalAprilTagYPos = 0.5;
    public boolean moveToAprilTag = false;

    // Configuration Constants (Static for @Configurable)
    @Sorter(sort = 0) public static boolean FieldOriented = false; // is field oriented enabled
    @Sorter(sort = 1) public static double kP = 0.04, kI = 0.0, kD = 0.0005;  // pid control constants
    @Sorter(sort = 4) public static double ROT_TOLERANCE_DEG = 2; // is pid within the tolerance (no wiggle)
    @Sorter(sort = 5) public static double MIN_ROT_POWER = 0.01; // minimum rotation power to overcome friction
    @Sorter(sort = 6) public static double VISION_TIMEOUT_MS = 150; // how long to keep using last known target

    public void initialize(HardwareMap hwMap) {
        // Motor Setup
        fl = hwMap.get(DcMotorEx.class, "left_front_motor");
        fr = hwMap.get(DcMotorEx.class, "right_front_motor");
        bl = hwMap.get(DcMotorEx.class, "left_back_motor");
        br = hwMap.get(DcMotorEx.class, "right_back_motor");

        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        br.setDirection(DcMotorEx.Direction.REVERSE);
        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

        // Pinpoint Setup
        odo = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(44, 0, DistanceUnit.MM); // !ADD NEW ROBOT OFFSETS!!!
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU(); //reset odo pos

        //Limelight Setup
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Battery Voltage Sensor
        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        // PID Utils (currently only for heading alignment)
        headingPID = new PIDController(kP, kI, kD, false);
        headingPID.setOutputMax(1);
    }


    public void updateOdo() {
        odo.update();
        TelemetryUtils.addData("Odo Status", odo.getDeviceStatus()); // send status to telemetry

        Pose2D pos = odo.getPosition(); // get current pos
        TelemetryUtils.addData("Odo x pos", pos.getX(DistanceUnit.MM)); // send positions to telemetry
        TelemetryUtils.addData("Odo y pos", pos.getY(DistanceUnit.MM));
        TelemetryUtils.addData("Odo heading", pos.getHeading(AngleUnit.DEGREES));
    }

    /**
     * Aligns the robot's heading to an AprilTag using Limelight feedback.
     *
     * @param maxMotorPower The maximum motor power to apply for rotation.
     * @return true if aligned within tolerance, false otherwise.
     */
    public boolean alignHeadingToAprilTag(double maxMotorPower) { // returns a bool: is aligned
        headingPID.setGains(kP, kI, kD); // update pid gains
        LLResult result = limelight.getLatestResult(); // gets all of limelight data

        double error;
        double ty;
        boolean targetFound;

        if (result.isValid()) {
            // if we see the tag Update our "Last Known" data
            error = result.getTx(); // how far off the target is from last position
            ty = result.getTy();
            lastKnownTx = error;
            lastKnownTy = ty;
            visionTimer.reset(); // Restart the "stale data" clock
            targetFound = true;
        } else if (visionTimer.milliseconds() < VISION_TIMEOUT_MS) {
            // We LOST the tag, but it was very recent. Use the last known error.
            error = lastKnownTx; // keep error as last known
            ty = lastKnownTy;
            targetFound = true; // Pretend we still have it for the PID
        } else {
            // Target has been gone too long. Stop moving.
            error = 0;
            ty = 0;
            targetFound = false;
        }

        // Calculate power using the PID (passing in our error)
        double rotPower = headingPID.calculate(error);

        // Apply constraints
        if (Math.abs(rotPower) < MIN_ROT_POWER) rotPower = 0; // if power is too small, don't move
        rotPower = Math.max(-maxMotorPower, Math.min(maxMotorPower, rotPower)); // clamp to max power (if neg, clamp to -max)

        boolean aligned = Math.abs(error) < ROT_TOLERANCE_DEG && result.isValid(); // check if aligned

        TelemetryUtils.addData("Vision Status", targetFound ? (result.isValid() ? "TRACKING" : "STALE DATA") : "LOST"); // send status to telemetry
        // Translation Powers = 0 because only aligning rotationally (for now)

        double motPower = 0;
      /*  if (moveToAprilTag) {
            if (ty<goalAprilTagYPos) {
                motPower = 0.5;
            } else if (ty>goalAprilTagYPos)
        }*/

        if(targetFound){
            applyMotorPower(0, 0, rotPower);
        } else {
            applyMotorPower(0, 0, 0); // Explicitly command motors to stop
        }

        return aligned;
    }


    /**
     * Helper method to consolidate the motor power math
     *
     */
    private void applyMotorPower(double y, double x, double rx) {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        // We cap the voltage at 12 to ensure we don't try to "overdrive"
        // when the battery is low, which could cause brownouts.
        double voltageScale = 12.0 / currentVoltage;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double scale = voltageScale / denominator;

        fl.setPower((y + x + rx) * scale);
        fr.setPower((y - x - rx) * scale);
        bl.setPower((y - x + rx) * scale);
        br.setPower((y + x - rx) * scale);
    }

    /**
     * Main method for driving the robot using a gamepad.
     * Implements field and robot oriented control and heading lock functionality.
     *
     * @param controller The gamepad to read inputs from.
     */
    public void gamepadDrive(Gamepad controller) {
        // Get raw values and apply deadband/cubing
        double rawX = (Math.abs(controller.left_stick_x) > 0.05) ? controller.left_stick_x : 0;
        double rawY = (Math.abs(controller.left_stick_y) > 0.05) ? -controller.left_stick_y : 0;
        double rawRx = (Math.abs(controller.right_stick_x) > 0.05) ? controller.right_stick_x : 0;
        // Exponential Shaping (Cubing the inputs) to make robot nicer to drive
        // Cubing preserves the negative/positive sign automatically
        double x = Math.pow(rawX, 3);
        double y = Math.pow(rawY, 3);
        double rx = Math.pow(rawRx, 3);
        if (acceleration) {
            x = x/2;
            y = y/2;
        }

        double currentHeadingDeg = Math.toDegrees(getHeading());

//        // Heading Lock Logic
//        if (Math.abs(rx) > 0.01) {
//            // Driver is actively turning: Update target to current heading
//            targetHeading = currentHeadingDeg;
//            isLocking = false;
//        } else if (Math.abs(x) > 0.01 || Math.abs(y) > 0.01) {
//            // Driver is moving but NOT turning: Use PID to maintain heading
//            double error = angleWrap(targetHeading - currentHeadingDeg);
//            rx = headingPID.calculate(error);
//            isLocking = true;
//        } else {
//            isLocking = false;
//        }

        if (FieldOriented) {
            double botHeading = getHeading();
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); // rotated X
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading); // rotated Y
            // Slightly increase rotation power to counteract added friction during strafing
            applyMotorPower(rotY, rotX * 1.1, rx);
        } else {
            applyMotorPower(y, x, rx);
        }
        TelemetryUtils.addData("Heading Lock", isLocking ? "ON" : "OFF");
        TelemetryUtils.addData("Target Heading", targetHeading);

    }

    @SuppressLint("DefaultLocale")
    public void logMotorCurrent() {
        double flC = fl.getCurrent(CurrentUnit.AMPS);
        double frC = fr.getCurrent(CurrentUnit.AMPS);
        double blC = bl.getCurrent(CurrentUnit.AMPS);
        double brC = br.getCurrent(CurrentUnit.AMPS);

        double totalCurr = flC + frC + blC + brC;

        // Log individual for debug diagnostics
        TelemetryUtils.debug("FL/FR Amps", String.format("%.1f / %.1f", flC, frC));
        TelemetryUtils.debug("BL/BR Amps", String.format("%.1f / %.1f", blC, brC));
        TelemetryUtils.addData("Total Drive Amps", totalCurr);

        // Check for whole-robot stall (Average)
        if ((totalCurr / 4.0) > 8.0) {
            TelemetryUtils.addData("WARNING", "ROBOT STALL DETECTED");
        }

        // Check for single-motor failure (Individual)
        // If one motor is significantly higher than the average, it's a mechanical bind
        if (flC > 10.0 || frC > 10.0 || blC > 10.0 || brC > 10.0) {
            TelemetryUtils.addData("CRITICAL", "SINGLE MOTOR OVERLOAD");
        }
    }

    public double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }


    public void switchOrientation () {
        FieldOriented = !FieldOriented;
    }

    public double getHeading() {
        // Get heading from pinpoint odometry in radians
        return odo.getPosition().getHeading(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        odo.resetPosAndIMU(); // Resets pinpoint heading to 0

    }
}