package org.firstinspires.ftc.teamcode;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="FieldOrientedDrivingTestOp")

public class FieldOrientedDrivingTestOp extends LinearOpMode {
    private GoBildaPinpointDriver odomhub;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor FR;


    @Override
    public void runOpMode() {

        odomhub = hardwareMap.get(GoBildaPinpointDriver.class,"odomhub");

        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL.setDirection(DcMotor.Direction.REVERSE); //so I don't have to think about
        BL.setDirection(DcMotor.Direction.REVERSE); //inverting later
        FR.setDirection(DcMotor.Direction.FORWARD); //should generally do whenever motors
        BR.setDirection(DcMotor.Direction.FORWARD);

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

            double currentrelativeheading = odomhub.getHeading(AngleUnit.RADIANS);
            //The rotational matrix multiplication curtisy of GPT
            double targetdrivey = leftstickinputx*cos(currentrelativeheading)-leftstickinputy*sin(currentrelativeheading);
            double targetdrivex = leftstickinputx*sin(currentrelativeheading)+leftstickinputy*cos(currentrelativeheading);


            double BRmotorpower = targetdrivey+targetdrivex-targetturn;
            double BLmotorpower = targetdrivey-targetdrivex+targetturn;
            double FRmotorpower = (targetdrivey-targetdrivex)-targetturn;
            double FLmotorpower = targetdrivey+targetdrivex+targetturn;


            odomhub.update();
            //assigns power to each motor based on gamepad inputs
            BR.setPower(BRmotorpower);
            BL.setPower(BLmotorpower);
            FR.setPower(FRmotorpower);
            FL.setPower(FLmotorpower);

            telemetry.addData(  "Status", "Running");
            telemetry.addData("rotation perceved",currentrelativeheading);
            telemetry.update();
        }
    }
}
