package org.firstinspires.ftc.teamcode.launcher;

import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.TeamColorRED;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottranslationx;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottranslationy;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.setRobotTargetYaw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class AutonoumusAutoLaunch extends LinearOpMode {


    public static double firingpowermultiplierconst = 3;
    public static void initializeLauncher(DcMotorEx LauncherFL,DcMotorEx LauncherFR) {




        //zeros the encoders and sets the run using encoder mode
        LauncherFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LauncherFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LauncherFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LauncherFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /**
     * Estimates the real-world width of an object using pixel width, camera FOV, resolution, and distance.
     *
     *
     *
     * @param Launcher LauncherFL
     * @param DrumServo the servo controlling the drum
     * @param FiringPinServo the servo controlling the pushing ball mechanism
     * @param TargetBallColor color of the ball that is being launched, 1 is green, 2 is purple
     *
     * @return output x, y, and turn motor power values
     */


    public static double  autoLaunchWithAim(DcMotorEx Launcher, Servo DrumServo, Servo FiringPinServo, double TargetBallColor, double[] drumBallColors){
        double ShootTargetY;
        double ShootTargetX = -3.6576/2.1;
        if( !TeamColorRED) {
            ShootTargetY = -3.6576/2.1;
        } else{
            ShootTargetY = 3.6576/2.1;
        }
        double targetturnangle = (57.29578 * ((Math.atan2(ShootTargetY - robottranslationy, ShootTargetX - robottranslationx))));
        setRobotTargetYaw(targetturnangle);
        double firingpower = (getFiringDistancethe(ShootTargetX,ShootTargetY) * firingpowermultiplierconst);



        /*double[] drumLocations = {0.1, 0.42, 0.76};// should probably make the drumb slots into objects
        int i = 0;
        for (double drumSlot: drumBallColors) {//slot finding loop
            if (drumSlot == TargetBallColor) {
                DrumServo.setPosition(drumLocations[i]);
                break;
            }
            if (i > 2) {
                break;
            }
            i++;
        }*/







        if (firingpower < 6) firingpower += 0.15;
        return(firingpower + 3.25);



    }
    public static double getFiringDistancethe(double targetx , double targety){
        double xdistance = Math.abs(targetx - robottranslationx);
        double ydistance = Math.abs(targety - robottranslationy);
        //pythangroniaun theorum to determine the distance
        return (Math.sqrt((xdistance * xdistance) + (ydistance * ydistance)));
    }

}
