package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.positioning.odometry.FieldOrientedDriving;

@TeleOp(name="LauncherDriverOPModeCrosby")

public class LauncherDriverOPModeCrosby extends LinearOpMode {
    private GoBildaPinpointDriver odomhub;
    private DcMotor Scooper;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor FR;


    @Override
    public void runOpMode() {

        odomhub = hardwareMap.get(GoBildaPinpointDriver.class,"odomhub");

        Scooper = hardwareMap.get(DcMotor.class,"Scooper");

        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL.setDirection(DcMotor.Direction.REVERSE); //so I don't have to think about
        BL.setDirection(DcMotor.Direction.REVERSE); //inverting later
        FR.setDirection(DcMotor.Direction.FORWARD); //should generally do whenever motors
        BR.setDirection(DcMotor.Direction.FORWARD);

        DcMotorEx LauncherFL = hardwareMap.get(DcMotorEx.class, "LauncherFL");

        //DcMotorEx LauncherFR = hardwareMap.get(DcMotorEx.class, "LauncherFR");

        double motortargetspeedradians = 0;

        double currentleftmotorvelocity = 0;

        double currentrightmotorvelocity = 0;

        //zeros the encoders and sets the run using encoder mode
        //VariablePowerLauncherAbstract.initializeLauncher(LauncherFL,LauncherFR);
        LauncherFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LauncherFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odomhub.initialize();
        odomhub.resetPosAndIMU();   // resets encoders and IMU

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftstickinputy = gamepad1.left_stick_y; // Forward/backward negative because it's naturally inverted
            double leftstickinputx = gamepad1.left_stick_x; // side to side
            double targetturn  = gamepad1.right_stick_x/2; // Turning

            //increments the target speed with up and right while decrementing it with left and down
            motortargetspeedradians += gamepad1.dpadUpWasPressed() ? 1 : 0;
            motortargetspeedradians -= gamepad1.dpadDownWasPressed() ? 1 : 0;
            motortargetspeedradians += gamepad1.dpadRightWasPressed() ? .1 : 0;
            motortargetspeedradians -= gamepad1.dpadLeftWasPressed() ? .1 : 0;

            double currentrelativeheading = odomhub.getHeading(AngleUnit.RADIANS);
            //Calls FieldOrientedDriving function and sets motor power
            double[] motorpowerarray = FieldOrientedDriving.fieldOrientedMath(leftstickinputy, leftstickinputx, targetturn, currentrelativeheading);

            double BRmotorpower = motorpowerarray[0];
            double BLmotorpower = motorpowerarray[1];
            double FRmotorpower = motorpowerarray[2];
            double FLmotorpower = motorpowerarray[3];

            // sets the velocity of the motors
            LauncherFL.setVelocity(motortargetspeedradians,AngleUnit.RADIANS);
            currentleftmotorvelocity = LauncherFL.getVelocity(AngleUnit.RADIANS);
            //currentrightmotorvelocity = LauncherFR.getVelocity(AngleUnit.RADIANS);
            double rawrightmotorvelocity = LauncherFL.getVelocity();



            telemetry.addLine("All Speeds are in Jacks Per Second");
            telemetry.addData("Motors' Target Rate of Rotation ", motortargetspeedradians);
            telemetry.addData("Left Motor Actual Rate of Rotation", currentleftmotorvelocity);
            //telemetry.addData("Right Motor Actual Rate of Rotation", currentrightmotorvelocity);
            telemetry.addData("rightmotorraw", rawrightmotorvelocity);
            telemetry.addData("Left Motor difference in Rate of Rotation", motortargetspeedradians-currentleftmotorvelocity);
            //telemetry.addData("Right Motor difference in Rate of Rotation", motortargetspeedradians+currentrightmotorvelocity);
            //telemetry.addData("Left Motor Speed at Wheel Surface meters per second",currentleftmotorvelocity*launcherwheelradiusm);
            //telemetry.addData("Right Motor Speed at Wheel Surface meters per second",currentrightmotorvelocity*launcherwheelradiusm);



            odomhub.update();
            //assigns power to each motor based on gamepad inputs
            BR.setPower(BRmotorpower);
            BL.setPower(BLmotorpower);
            FR.setPower(FRmotorpower);
            FL.setPower(FLmotorpower);



            telemetry.addData(  "Status", "Running");
            telemetry.addData("rotation perceived",currentrelativeheading);
            telemetry.update();
        }
    }
}
