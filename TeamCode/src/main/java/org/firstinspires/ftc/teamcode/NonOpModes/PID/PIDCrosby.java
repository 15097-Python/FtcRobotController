package org.firstinspires.ftc.teamcode.NonOpModes.PID;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottranslationx;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottranslationy;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robotyaw;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottargetx;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottargety;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottargetyaw;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public abstract class PIDCrosby {


    //defining variables for pid
    static double xoutputMotorPower = 0;
    static final double xproportionalConstant = 0.005;
    static double xcurrentError = 0;
    static double xtotalError = 0;
    static double xlastError = 0;
    static final double xintegralConstant = 0;
    static final double xderivativeConstant = 0.001;
    static double youtputMotorPower = 0;
    static final double yproportionalConstant = 0.005;
    static double ycurrentError = 0;
    static double ytotalError = 0;
    static double ylastError = 0;
    static final double yintegralConstant = 0;
    static final double yderivativeConstant = 0.001;
    static double turnoutputMotorPower = 0;
    static final double turnproportionalConstant = 0.05;
    static double turncurrentError = 0;
    static double turntotalError = 0;
    static double turnlastError = 0;
    static final double turnintegralConstant = 0;
    static final double turnderivativeConstant = 0.5;
    /**
     *
     * @param timeBetweenLoops Time between opmodeloops
     * @return output x, y, and turn motor power values
     */
    public static double settingMotorPIDPowerX(Double timeBetweenLoops){

        xcurrentError = robottargetx-robottranslationx;
        xtotalError += xcurrentError;

        xoutputMotorPower = (xproportionalConstant * xcurrentError) + (xintegralConstant * xtotalError) + (xderivativeConstant * (xcurrentError - xlastError)/timeBetweenLoops);
        xlastError = xcurrentError;
        return xoutputMotorPower;
    }
    public static double settingMotorPIDPowerY(Double timeBetweenLoops){
        ycurrentError = robottargety-robottranslationy;
        ytotalError += ycurrentError;
        youtputMotorPower = (yproportionalConstant * ycurrentError) + (yintegralConstant * ytotalError) + (yderivativeConstant * (ycurrentError - ylastError)/timeBetweenLoops);
        ylastError = ycurrentError;
        return youtputMotorPower;
    }
    public static double settingMotorPIDPowerYaw(Double timeBetweenLoops){
        turncurrentError = AngleUnit.normalizeDegrees( robottargetyaw-robotyaw);
        turntotalError += turncurrentError;
        turnoutputMotorPower = (turnproportionalConstant * turncurrentError) + (turnintegralConstant * turntotalError) + (turnderivativeConstant * (turncurrentError - turnlastError)/timeBetweenLoops);
        turnlastError = turncurrentError;
        return turnoutputMotorPower;
    }
}


