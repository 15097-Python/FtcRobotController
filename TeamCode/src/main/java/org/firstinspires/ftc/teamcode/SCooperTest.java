package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.positioning.odometry.FieldOrientedDriving;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="ScooperTest")

public class SCooperTest extends LinearOpMode {
    private GoBildaPinpointDriver odomhub;
    private DcMotorEx Scooper;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor FR;


    @Override
    public void runOpMode() {

        odomhub = hardwareMap.get(GoBildaPinpointDriver.class,"odomhub");

        Scooper = hardwareMap.get(DcMotorEx.class,"Scooper");
        Scooper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Scooper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            //Calls FieldOrientedDriving function and sets motor power
            double motorpowerarray[] = FieldOrientedDriving.fieldOrientedMath(leftstickinputy, leftstickinputx, targetturn, currentrelativeheading);


            double BRmotorpower = motorpowerarray[0];
            double BLmotorpower = motorpowerarray[1];
            double FRmotorpower = motorpowerarray[2];
            double FLmotorpower = motorpowerarray[3];


            odomhub.update();
            //assigns power to each motor based on gamepad inputs
            BR.setPower(BRmotorpower);
            BL.setPower(BLmotorpower);
            FR.setPower(FRmotorpower);
            FL.setPower(FLmotorpower);

            if (gamepad1.left_bumper) Scooper.setVelocity(999,AngleUnit.RADIANS);
            if (gamepad1.right_bumper) Scooper.setVelocity(999,AngleUnit.RADIANS);

            telemetry.addData(  "Status", "Running");
            telemetry.addData("rotation perceived",currentrelativeheading);
            telemetry.update();
        }
    }
}
