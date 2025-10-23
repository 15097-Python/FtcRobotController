package org.firstinspires.ftc.teamcode.NonOpModes.PID;


public abstract class PIDCrosby {


    //defining variables for pid
    static double xoutputMotorPower = 0;
    static final double xproportionalConstant = 0;
    static double xcurrentError = 0;
    static double xtotalError = 0;
    static double xlastError = 0;
    static final double xintegralConstant = 0;
    static final double xderivativeConstant = 0;
    static double youtputMotorPower = 0;
    static final double yproportionalConstant = 0;
    static double ycurrentError = 0;
    static double ytotalError = 0;
    static double ylastError = 0;
    static final double yintegralConstant = 0;
    static final double yderivativeConstant = 0;
    static double turnoutputMotorPower = 0;
    static final double turnproportionalConstant = 0;
    static double turncurrentError = 0;
    static double turntotalError = 0;
    static double turnlastError = 0;
    static final double turnintegralConstant = 0;
    static final double turnderivativeConstant = 0;
    /**
     * Estimates the real-world width of an object using pixel width, camera FOV, resolution, and distance.
     *
     * @param xTargetCoordinate   Target coordinate in meters
     * @param xCurrentCoordinate   Current coordinate in meters
     * @param yTargetCoordinate Target coordinate in meters
     * @param yCurrentCoordinate    Current Coordinate in meters
     * @param turnTargetRadian Target angle in radians
     * @param turnCurrentRadian Current angle in radians
     * @param timeBetweenLoops Time between opmodeloops
     * @return output x, y, and turn motor power values
     */
        public static double[] settingMotorPIDPower(Double xTargetCoordinate, Double xCurrentCoordinate, Double yTargetCoordinate, Double yCurrentCoordinate, Double turnCurrentRadian, Double turnTargetRadian, Double timeBetweenLoops){
        if (xTargetCoordinate == null) xTargetCoordinate = 0.0;
        if (xCurrentCoordinate == null) xCurrentCoordinate = 0.0;
        if (yTargetCoordinate == null) yTargetCoordinate = 0.0;
        if (yCurrentCoordinate == null) yCurrentCoordinate = 0.0;
        if (turnTargetRadian == null) turnTargetRadian = 0.0;
        if (turnCurrentRadian == null) turnCurrentRadian = 0.0;

        xcurrentError = xTargetCoordinate-xCurrentCoordinate;
        ycurrentError = yTargetCoordinate-yCurrentCoordinate;
        turncurrentError = turnTargetRadian-turnCurrentRadian;
        xtotalError+=xcurrentError;
        ytotalError+=ycurrentError;
        turntotalError+=turncurrentError;

        xoutputMotorPower = (xproportionalConstant * xcurrentError) + (xintegralConstant * xtotalError) + (xderivativeConstant * (xcurrentError - xlastError)/timeBetweenLoops);
        youtputMotorPower = (yproportionalConstant * ycurrentError) + (yintegralConstant * ytotalError) + (yderivativeConstant * (ycurrentError - ylastError)/timeBetweenLoops);
        turnoutputMotorPower = (turnproportionalConstant * turncurrentError) + (turnintegralConstant * turntotalError) + (turnderivativeConstant * (turncurrentError - turnlastError)/timeBetweenLoops);
        xlastError=xcurrentError;
        ylastError=ycurrentError;
        turnlastError=turncurrentError;
        double[] outputarray = {xoutputMotorPower,youtputMotorPower,turnoutputMotorPower};
        return (outputarray);
    }
}

