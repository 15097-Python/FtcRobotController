package org.firstinspires.ftc.teamcode.Util;

public class RobotPositionCrosby {
        // The robot's field position (x, y, z, rotation)
        //Red is True
        public static boolean TeamColorRED = true;
        public static double robottranslationx = 0;//translation
        public static double robottargetx = 0;
        public static double robottranslationy = 0;
        public static double robottargety = 0;
        public static double robottranslationz = 0;
        public static double robottargetz = 0;

        public static double robotroll = 0;//rotations
        public static double robottargetroll = 0;
        public static double robotpitch = 0;
        public static double robottargetpitch = 0;
        public static double robotyaw = 0;
        public static double robottargetyaw = 0;


        public static void setRobotTargetYaw(double newtargetyaw){
            robottargetyaw = newtargetyaw;
        }
        /**
         * grabs the coordinates of the robot
         * @return the orter of returned values is transx transy transz roll pitch yaw
         */
        public static double[] getRobotCoordinates() {
            return new double[] {robottranslationx,robottranslationy,robottranslationz,robotroll,robotpitch,robotyaw};
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
