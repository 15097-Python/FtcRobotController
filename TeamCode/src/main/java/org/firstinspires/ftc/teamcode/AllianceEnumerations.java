package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public enum AllianceEnumerations {
    RED,
    BLUE;
    public static final double FIELD_LENGTH = 3.6576;
    public static final double FIELD_WIDTH = 3.6576;

    // Flip a Pose2d based on the alliance color
    public Pose2d flipPose(Pose2d pose) {

        if (this == BLUE) {
            // flip logic
            double origX = pose.position.x;
            double origY = pose.position.y;
            double origHeading = pose.heading.toDouble();

            double flippedX = FIELD_LENGTH - origX;
            double flippedY = FIELD_WIDTH - origY;
            double flippedHeading = origHeading + Math.toRadians(180);

            return new Pose2d(flippedX, flippedY, flippedHeading);
        } else {
            return pose; // No flip for RED alliance
        }
    }

    // Convert a string to the corresponding AllianceEnumerations value
    public static AllianceEnumerations fromString(String s) {
        if (s == null) {
            throw new IllegalArgumentException("Alliance string is null");
        }
        switch (s.toUpperCase()) {
            case "RED": return RED;
            case "BLUE": return BLUE;
            default: throw new IllegalArgumentException("Unknown alliance: " + s);
        }
    }

    @Override
    public String toString() {
        return this.name().toLowerCase();
    }
}
