package org.firstinspires.ftc.teamcode.Util.Enum;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Pose2d;

public enum AllianceEnumerations {
    RED,
    BLUE;

    public static final double FIELD_SIZE = 3.6576; // Same for length/width in FRC/FTC fields

    /** Whether this alliance should flip coordinates */
    public boolean shouldFlip() {
        return this == BLUE;
    }

    /**Flip a Pose2d based on the alliance color */
    public Pose2d flipPose(Pose2d pose) {

        if (this == BLUE) {
            // flip logic
            double origX = pose.position.x;
            double origY = pose.position.y;
            double origHeading = pose.heading.toDouble();

            double flippedX = FIELD_SIZE - origX;
            //double flippedY = FIELD_SIZE - origY;
            double flippedHeading = origHeading + Math.toRadians(180);

            return new Pose2d(flippedX, origY, flippedHeading);
        } else {
            return pose; // No flip for RED alliance
        }
    }

    /** Flip a single 1D coordinate (for odometry or line mapping) */
    public double flip1D(double positionX) {
        return shouldFlip() ? FIELD_SIZE - positionX : positionX;
    }

    /** Parse a string to an alliance enum */
    public static AllianceEnumerations fromString(String s) {
        if (s == null) throw new IllegalArgumentException("Alliance string is null");
        switch (s.toUpperCase()) {
            case "RED": return RED;
            case "BLUE": return BLUE;
            default: throw new IllegalArgumentException("Unknown alliance: " + s);
        }
    }

    @NonNull
    @Override
    public String toString() {
        return name().toLowerCase();
    }
}
