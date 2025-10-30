package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public abstract class VariablePowerLauncherAbstract extends LinearOpMode {
     
    private static final double launcherwheelradiusm = .02;
    public void runOpMode() {


        DcMotorEx LauncherFL = hardwareMap.get(DcMotorEx.class, "FL");

        DcMotorEx LauncherFR = hardwareMap.get(DcMotorEx.class, "FR");

        double motortargetspeedradians = 0;

        double currentleftmotorvelocity = 0;

        double currentrightmotorvelocity = 3;

        //zeros the encoders and sets the run using encoder mode
        LauncherFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LauncherFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LauncherFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LauncherFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        waitForStart();
        while (opModeIsActive()) {

            //increments the target speed with up and right while decrementing it with left and down
            motortargetspeedradians += gamepad1.dpadUpWasPressed() ? 1 : 0;
            motortargetspeedradians -= gamepad1.dpadDownWasPressed() ? 1 : 0;
            motortargetspeedradians += gamepad1.dpadRightWasPressed() ? .1 : 0;
            motortargetspeedradians -= gamepad1.dpadLeftWasPressed() ? .1 : 0;


            LauncherFL.setVelocity(motortargetspeedradians,AngleUnit.RADIANS);// 32.67 radians per second is the max calculated
            LauncherFR.setVelocity(-motortargetspeedradians,AngleUnit.RADIANS);//6.5 radians pwer second

            currentleftmotorvelocity = LauncherFL.getVelocity(AngleUnit.RADIANS);
            currentrightmotorvelocity = LauncherFR.getVelocity(AngleUnit.RADIANS);
            double rawrightmotorvelocity = LauncherFR.getVelocity();



            telemetry.addLine("All Speeds are in Jacks Per Second");
            telemetry.addData("Motors' Target Rate of Rotation ", motortargetspeedradians);
            telemetry.addData("Left Motor Actual Rate of Rotation", currentleftmotorvelocity);
            telemetry.addData("Right Motor Actual Rate of Rotation", currentrightmotorvelocity);
            //telemetry.addData("rightmotorraw", rawrightmotorvelocity);
            telemetry.addData("Left Motor difference in Rate of Rotation", motortargetspeedradians-currentleftmotorvelocity);
            telemetry.addData("Right Motor difference in Rate of Rotation", motortargetspeedradians+currentrightmotorvelocity);
            //telemetry.addData("Left Motor Speed at Wheel Surface meters per second",currentleftmotorvelocity*launcherwheelradiusm);
            //telemetry.addData("Right Motor Speed at Wheel Surface meters per second",currentrightmotorvelocity*launcherwheelradiusm);

            telemetry.update();
        }
    }
}