package org.firstinspires.ftc.teamcode.launcher;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public abstract class VariablePowerLauncherAbstract extends LinearOpMode {
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
     * @param currentrightmotorvelocity the current velocity of the right motor
     * @param LauncherFL front left launcher
     * @param LauncherFR front right launcher
     * @return output x, y, and turn motor power values
     */


        public static void  launcherFunction(double motortargetspeedradians, double currentleftmotorvelocity, double currentrightmotorvelocity, DcMotorEx LauncherFL, DcMotorEx LauncherFR){




            LauncherFL.setVelocity(motortargetspeedradians,AngleUnit.RADIANS);
            LauncherFR.setVelocity(-motortargetspeedradians,AngleUnit.RADIANS);


        }
    }
