package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="VariablePowerLauncherTellyOP")

public class VariablePowerLauncherTellyOP extends LinearOpMode {
     
    private static final double launcherwheelradiusm = .02;


    @Override
    public void runOpMode() {

        DcMotorEx FL = hardwareMap.get(DcMotorImplEx.class, "FL");

        DcMotorEx FR = hardwareMap.get(DcMotorImplEx.class, "FR");

        double motortargetspeedradians = 0;

        double currentleftmotorvelocity = 0;

        double currentrightmotorvelocity = 3;

        //zeros the encoders and sets the run using encoder mode
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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


            FL.setVelocity(motortargetspeedradians,AngleUnit.RADIANS);// 32.67 radians per second is the max calculated
            FR.setVelocity(-motortargetspeedradians,AngleUnit.RADIANS);//6.5 radians pwer second

            currentleftmotorvelocity = FL.getVelocity(AngleUnit.RADIANS);

            currentrightmotorvelocity = FR.getVelocity(AngleUnit.RADIANS);

            if (currentleftmotorvelocity == 0){
                telemetry.addLine("returning 0 left ");
            }
            if (currentrightmotorvelocity == 0){
                telemetry.addLine("returning 0 right ");
            }

            telemetry.addLine("All Speeds are in Radians Per Second");
            telemetry.addData("Motors' Target Rate of Rotation ", motortargetspeedradians);
            telemetry.addData("Left Motor Actual Rate of Rotation", currentleftmotorvelocity);
            telemetry.addData("Right Motor Actual Rate of Rotation", currentrightmotorvelocity);
            telemetry.addData("Left Motor difference in Rate of Rotation", motortargetspeedradians-currentleftmotorvelocity);
            telemetry.addData("Right Motor difference in Rate of Rotation", motortargetspeedradians+currentrightmotorvelocity);
            telemetry.addData("Left Motor Speed at Wheel Surface",currentleftmotorvelocity*launcherwheelradiusm);
            telemetry.addData("Right Motor Speed at Wheel Surface",currentrightmotorvelocity*launcherwheelradiusm);

            telemetry.update();
        }
    }
}