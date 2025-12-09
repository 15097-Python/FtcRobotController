package org.firstinspires.ftc.teamcode.launcher;

import static org.firstinspires.ftc.teamcode.Util.RobotPosition.TeamColorRED;
import static org.firstinspires.ftc.teamcode.Util.RobotPosition.robottranslationx;
import static org.firstinspires.ftc.teamcode.Util.RobotPosition.robottranslationy;
import static org.firstinspires.ftc.teamcode.Util.RobotPosition.setRobotTargetYaw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public abstract class SemiAutoAiming extends LinearOpMode {


    public static double firingpowermultiplierconst = 3;
    private final static double radiansToDegrees = 57.29578;

    public static void  setAimAngle(){
        double ShootTargetY;
        double ShootTargetX = -3.6576/2.1;
        if( !TeamColorRED) {
            ShootTargetY = -3.6576/2.1;
        } else{
            ShootTargetY = 3.6576/2.1;
        }
        //chagpt wizardry
        double targetturnangle = (radiansToDegrees * ((Math.atan2(ShootTargetY - robottranslationy, ShootTargetX - robottranslationx))));
        setRobotTargetYaw(targetturnangle);
    }
    public static double siezingMotorTargetRotation(){
        double ShootTargetY;
        double ShootTargetX = -3.6576/2.1;
        if( !TeamColorRED) {
            ShootTargetY = -3.6576/2.1;
        } else{
            ShootTargetY = 3.6576/2.1;
        }
        double xdistance = Math.abs(ShootTargetX - robottranslationx);
        double ydistance = Math.abs(ShootTargetY - robottranslationy);

        //pythangroniaun theorum to determine the distance
        double distance = Math.sqrt((xdistance * xdistance) + (ydistance * ydistance)) * firingpowermultiplierconst + 3;
        if (distance < 9) distance += 0.15;
        return (distance);
    }

}
