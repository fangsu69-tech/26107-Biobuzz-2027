package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResult;

public class LimelightUtils {

    /**
     * Calculates the rotation power needed to align to a target.
     * Returns 0.0 if no target is found or if aligned.
     */
    public static double calculateRotationPower(LLResult result, PIDController controller,
                                                double maxPower, double minPower, double tolerance) {

        if (result == null || !result.isValid()) {
            controller.reset();
            return 0.0;
        }

        double error = -result.getTx();

        // If within tolerance, stop rotating
        if (Math.abs(error) < tolerance) {
            controller.reset();
            return 0.0;
        }

        double power = controller.calculate(error);

        // Clamp to max power
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Apply deadband (minimum power to overcome friction)
        if (Math.abs(power) < minPower) {
            power = Math.signum(power) * minPower;
        }

        return power;
    }

    public static boolean isAligned(LLResult result, double tolerance) {
        return result != null && result.isValid() && Math.abs(result.getTx()) < tolerance;
    }
}