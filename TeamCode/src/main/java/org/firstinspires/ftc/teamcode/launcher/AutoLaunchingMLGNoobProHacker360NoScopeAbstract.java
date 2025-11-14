package org.firstinspires.ftc.teamcode.launcher;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.TeamColorRED;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottranslationx;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottranslationy;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robotyaw;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.AllianceEnumerations;


public abstract class AutoLaunchingMLGNoobProHacker360NoScopeAbstract extends LinearOpMode {
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
     * @param motortargetspeedradians   The target motor speed in radians
     * @param currentleftmotorvelocity   the current velocity of the left motor
     * @param LauncherFL front left launcher
     * @param DrumServo the servo controlling the drum
     * @param FiringPinServo the servo controlling the pushing ball mechanism
     * @param TargetBallColor color of the ball that is being launched, 1 is green, 2 is purple
     * @param AprilTagDistance distance from the april tag
     * @return output x, y, and turn motor power values
     */


    public static void  AutoLaunch(double motortargetspeedradians, double currentleftmotorvelocity, DcMotorEx LauncherFL, Servo DrumServo, Servo FiringPinServo, double TargetBallColor, double AprilTagDistance, double[] drumBallColors){
        final double firingpowermultiplierconst = 2;
        double ShootTargetX = 0;
        double ShootTargetY = 3.5;
        if(TeamColorRED == false) {
            ShootTargetX = 3.5;
        } else{
            ShootTargetX = -3.5;
        }
        double firingpower = (getFiringDistance(ShootTargetX,ShootTargetY) * firingpowermultiplierconst);
        double shootingangle = Math.atan2(ShootTargetY - robottranslationy, ShootTargetX - robottranslationx);//chatgpt math


        double[] drumLocations = {0.2, 0.5, 0.8};// should probably make the drumb slots into objects
        int i = 0;
        for (double drumSlot: drumBallColors) {//slot finding loop
            if (drumSlot == TargetBallColor) {
                DrumServo.setPosition(drumLocations[i]);
                break;
            }
            if (i > 2) {
                return;
            }
            i++;
        }








        LauncherFL.setVelocity(firingpower,AngleUnit.RADIANS);


    }
    public static double getFiringDistance(double targetx , double targety){
        double xdistance = Math.abs(targetx - robottranslationx);
        double ydistance = Math.abs(targety - robottranslationy);
        //pythangroniaun theorum to determine the distance
        double distance = Math.sqrt((xdistance * xdistance) + (ydistance * ydistance));
        return (distance);
    }
}
