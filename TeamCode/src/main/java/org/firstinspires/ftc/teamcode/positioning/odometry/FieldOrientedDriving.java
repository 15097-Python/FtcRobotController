package org.firstinspires.ftc.teamcode.positioning.odometry;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public abstract class FieldOrientedDriving {
    /**
     * Estimates the real-world width of an object using pixel width, camera FOV, resolution, and distance.
     *
     * @param leftstickinputx   the left stick's x input
     * @param leftstickinputy   the left stick's y input
     * @param targetturn the right stick's x value
     * @param currentrelativeheading the gyro scope's value based off of the initial heading which eventualy sholud be based off of the rotation of the robot from the limelight
     * @return output x, y, and turn motor power values
     */
    public static double[] fieldOrientedMath(double leftstickinputx, double leftstickinputy, double targetturn, double currentrelativeheading){
        double targetdrivey = leftstickinputx*cos(currentrelativeheading)-leftstickinputy*sin(currentrelativeheading);
        double targetdrivex = leftstickinputx*sin(currentrelativeheading)+leftstickinputy*cos(currentrelativeheading);


        double BRmotorpower = targetdrivey+targetdrivex-targetturn;
        double BLmotorpower = targetdrivey-targetdrivex+targetturn;
        double FRmotorpower = (targetdrivey-targetdrivex)-targetturn;
        double FLmotorpower = targetdrivey+targetdrivex+targetturn;

        return new double[]{BRmotorpower, BLmotorpower, FRmotorpower, FLmotorpower};
    }
}
