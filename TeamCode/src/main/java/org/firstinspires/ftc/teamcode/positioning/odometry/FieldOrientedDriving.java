package org.firstinspires.ftc.teamcode.positioning.odometry;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public abstract class FieldOrientedDriving {
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
