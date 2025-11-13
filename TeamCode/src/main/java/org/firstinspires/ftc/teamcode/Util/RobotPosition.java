package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class RobotState {
    // The robot's field position (x, y, z, rotation)
    private static double robottranslationx = 0;//translation
    private static double robottranslationy = 0;
    private static double robottranslationz = 0;
    
    private static double robotroll = 0;//rotations
    private static double robotpitch = 0;
    private static double robotyaw = 0;



    /**
     * grabs the coordinates of the robot
     * @return the orter of returned values is transx transy transz roll pitch yaw
     */
    public static double[] getRobotCoordinates() {
        return new double[] {robottranslationx,robottranslationy,robottranslationz,robotroll,robotpitch,robotyaw}
    }

    /**
     * sets the robot's translations from the orogin
     *
     * @param newx   the left stick's x input
     * @param newy   the left stick's y input
     * @param newz   the right stick's x value
     * @param newroll   the left stick's x input
     * @param newpitch   the left stick's y input
     * @param newyaw   the right stick's x value
     */
    public static void modifyRobotCoordinates(Double newx, Double newy, Double newz, Double newroll, Double newpitch, Double newyaw) {
        if (newx != null) robottranslationx = newx;
        if (newy != null) robottranslationy = newy;
        if (newz != null) robottranslationz = newz;

        if (newroll != null) robotroll = newroll;
        if (newpitch != null) robotpitch = newpitch;
        if (newyaw != null) robotyaw = newyaw;
    }

}
