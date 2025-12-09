package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="controllerTestOPCrosby")
@Disabled
public class controllerTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL"); // local hardware mapping
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.FORWARD); //so I don't have to think about
        BL.setDirection(DcMotor.Direction.FORWARD); //inverting later
        FR.setDirection(DcMotor.Direction.REVERSE); //should generally do whenever motors
        BR.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y; // Forward/backward negative because it's naturally inverted
            double turn  = gamepad1.right_stick_x; // Turning

            double leftPower  = drive + turn; // helps with turning
            double rightPower = drive - turn; // left and right

            leftPower  = Math.max(-1.0, Math.min(1.0, leftPower)); //constrains the # cause these are
            rightPower = Math.max(-1.0, Math.min(1.0, rightPower)); //the max and min allowed

            FL.setPower(leftPower); //tells the motors how fast to go
            BL.setPower(leftPower);
            FR.setPower(rightPower);
            BR.setPower(rightPower);

            telemetry.addData("FL/BL Power: ", leftPower); //tells us how much power the motors are using
            telemetry.addData("FR/BR Power: ", rightPower);
            telemetry.addData("Left Stick Y: ", (-gamepad1.left_stick_y)); //tells us what the stick values were
            telemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
            telemetry.addData("Right Trigger: ", gamepad1.right_trigger); //just wanted to see what this outputted
            telemetry.addData("A Button Pressed: ", gamepad1.a); //tells us what button pressed i think
            telemetry.update();
            }
        }
    }
