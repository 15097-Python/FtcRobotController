package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.teamcode.Util.constants.RobotStats.*;
import static org.firstinspires.ftc.teamcode.Util.constants.Physics.*;
import com.qualcomm.robotcore.util.RobotLog; // For error logging
import org.firstinspires.ftc.robotcore.external.Telemetry;

/*public class RPMEstimator {

    /**
     * Calculate required RPM for given distance and angle
     * @param distance   Horizontal distance to target (meters) found with limelight
     * @return Required RPM without taking offset into account
     */
/*    public static double calculateRequiredRPMOld(double distance) {
        // Calculate required launch velocity
        double velocity = Math.sqrt(distance * GRAVITY / Math.sin(2 * launchAngle));

        // Convert to RPM
        return (velocity * 60.0) / (2.0 * Math.PI * WheelRadius);
    }

    /**
     * Calculate required flywheel RPM to hit a target with 3D offsets
     * @param xDistance   Horizontal forward distance to target (meters)
     * @return Required RPM, or -1 if impossible with given angle
     */
/*    public static double calculateRequiredRPM(double xDistance) {

        // Check if trajectory is physically possible
        double denominator = xDistance * Math.tan(launchAngle) - ApriltagOffsetY;
        if (denominator <= EPSILON) {
            throw new IllegalArgumentException(
                    "Launch angle too shallow to reach target height. Try a steeper angle."
            );
        }

        // Calculate required launch velocity
        double numerator = GRAVITY * xDistance * xDistance;
        double velocity = Math.sqrt(numerator / (2 * Math.cos(launchAngle) * Math.cos(launchAngle) * denominator));

        // Convert linear velocity to angular velocity (RPM)

        return (velocity * 60.0) / (2.0 * Math.PI * WheelRadius);
    }

    /**
     * Calculate required horizontal aim angle to cancel Z offset
     * @param xDistance Forward distance (meters)
     * @return Horizontal aim adjustment in RADIANS
     */
/*    public static double calculateHorizontalAim(double xDistance) {
        return Math.atan2(ApriltagOffsetZ, xDistance);
    }

    /**
     * Check if RPM is within motor's operating range
     * @return true if feasible, false otherwise
     */
/*    public static boolean isRPMFeasible(double requiredRPM) {
        return requiredRPM >= MinRPM && requiredRPM <= MaxRPM;
    }
} */

public class RPMEstimator {

    // If you add external gearing AFTER the motor's output shaft, set this
    // Example: 20.0 means wheel spins 20× faster than the motor shaft
    private static final double EXTERNAL_SPEED_RATIO = 1.0; // CHANGE THIS IF NEEDED

    public static double wheelRPMToEncoderVelocity(double wheelRPM) {
        double shaftRPM = wheelRPM / EXTERNAL_SPEED_RATIO;
        double revolutionsPerSecond = shaftRPM / 60.0;
        return revolutionsPerSecond * ENCODER_TICKS_PER_OUTPUT_REV;
    }

    public static double calculateRequiredRPM(double xDistance) {
        double launchAngleRad = Math.toRadians(launchAngle);
        double denominator = xDistance * Math.tan(launchAngleRad) - ApriltagOffsetY;

        if (denominator <= EPSILON) {
            throw new IllegalArgumentException("Launch angle too shallow for target height.");
        }

        double numerator = GRAVITY * xDistance * xDistance;
        double velocity = Math.sqrt(numerator /
                (2 * Math.cos(launchAngleRad) * Math.cos(launchAngleRad) * denominator));

        return (velocity * 60.0) / (2.0 * Math.PI * WheelRadius);
    }

    public static void checkFeasibility(double xDistance, Telemetry telemetry) {
        double requiredRPM = calculateRequiredRPM(xDistance);
        boolean feasible = requiredRPM <= MaxRPM * 0.9;

        telemetry.addData("Target Distance", "%.2f m", xDistance);
        telemetry.addData("Required Wheel RPM", "%.0f", requiredRPM);
        telemetry.addData("Motor Max RPM", "%.0f", MaxRPM);
        telemetry.addData("Feasible?", feasible ? "YES" : "NO");

        if (!feasible) {
            double speedRatio = requiredRPM / MaxRPM;
            telemetry.addData("SOLUTION", "Add %.1fx gearing or faster motor", speedRatio);
        }
    }

    public static boolean isRPMFeasible(double requiredRPM) {
        return requiredRPM >= MinRPM && requiredRPM <= MaxRPM * 0.9;
    }

    public static double calculateHorizontalAim(double xDistance) {
        return Math.atan2(ApriltagOffsetZ, xDistance);
    }
}