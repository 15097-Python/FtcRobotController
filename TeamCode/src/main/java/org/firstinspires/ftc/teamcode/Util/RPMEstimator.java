package org.firstinspires.ftc.teamcode.Util;

import static org.firstinspires.ftc.teamcode.Util.constants.RobotStats.*;
import static org.firstinspires.ftc.teamcode.Util.constants.Conversions.*;

public class RPMEstimator {
    /**
     * Calculate required RPM for given distance and angle
     * @param distance   Horizontal distance to target (meters) found with limelight
     * @return Required RPM
     */
    public static double calculateRequiredRPM(double distance) {
        // Calculate required launch velocity
        double velocity = Math.sqrt(distance * Gravity / Math.sin(2 * launchAngle));

        // Convert to RPM
        return (velocity * 60.0) / (2.0 * Math.PI * WheelRadius);
    }

    /**
     * Check if RPM is within motor's operating range
     * @return true if feasible, false otherwise
     */
    public static boolean isRPMFeasible(double requiredRPM) {
        return requiredRPM >= MinRPM && requiredRPM <= MaxRPM;
    }
}


