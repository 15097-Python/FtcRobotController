package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;

public class poseFlip {
    // Field dimensions (meters)
    public static final double FIELD_LENGTH = 3.6576;
    public static final double FIELD_WIDTH = 3.6576;

    public static Pose2d flipPose(Pose2d pose, Alliance alliance) {
        if (alliance == Alliance.RED) {
            return pose;
        } else if (alliance == Alliance.BLUE) {
            // flip logic
            double origX = pose.position.x;
            double origY = pose.position.y;
            double origHeading = pose.heading.toDouble();

            double flippedX = FIELD_LENGTH - origX;
            double flippedY = FIELD_WIDTH - origY;
            double flippedHeading = origHeading + Math.toRadians(180);

            return new Pose2d(flippedX, flippedY, flippedHeading);
        } else {
            // In case a new enum is added, or null passed, just return pose as default
            return pose;
        }
    }

    public enum Alliance { RED, BLUE }
}
