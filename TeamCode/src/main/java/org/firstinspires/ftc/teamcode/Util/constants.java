package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;

/** (import static org.firstinspires.ftc.teamcode.Util.constants.*;)
 * import it by doing this and then you can just type something like
 * (EX: Drive.MAX_POWER)
 * or whatever you want to use
 * you could also do something like
 * (import static org.firstinspires.ftc.teamcode.Util.constants.motor.*;)
 * and then you can use any variable you need in the subset
 * (EX: MAX_POWER)
*/
public final class constants {
    /**
     * hardware map names
     */
    public static final class PART_NAMES{
        public static final String FL = "FL"; //despite what you may think these are constants
        public static final String FR = "FR"; //yes I know names are constant it's crazy
        public static final String BL = "BL";
        public static final String BR = "BR";
        public static final String DrumServo = "DrumServo";
        public static final String FiringPinServo = "FiringPinServo";
        public static final String odomhub = "odomhub";
        public static final String limelight = "limelight";

    }

    /**
     * general motor initialization steps and ranges
     */
    public static final class MOTOR{
        //remember to change these later to use actual numbers and explain them
        public static final double MAX_POWER = 0;
        public static final double MIN_POWER = 0;
        public static final DcMotor.Direction FL_DIR = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction BL_DIR = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction FR_DIR = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction BR_DIR = DcMotor.Direction.REVERSE;
    }

    /**
     * things that were constant in PID
     */
    public static final class PID{
        public static final double xproportionalConstant = 0;
        public static final double xintegralConstant = 0;
        public static final double xderivativeConstant = 0;
        public static final double yproportionalConstant = 0;
        public static final double yintegralConstant = 0;
        public static final double yderivativeConstant = 0;
        public static final double turnproportionalConstant = 0;
        public static final double turnintegralConstant = 0;
        public static final double turnderivativeConstant = 0;
    }

    /**
     * facts about field that do not change or should not
     * such as lengths
     */
    public static final class FIELD{
        public static final double FIELD_SIZE = 3.6576;
        public static final double FIELD_HALF = 3.6576/2;
    }

    /**
     * eventually servo stuff IDK
     */
    public static final class Conversions{
        public static final double InToM = 25.4*1000;
    }
}
