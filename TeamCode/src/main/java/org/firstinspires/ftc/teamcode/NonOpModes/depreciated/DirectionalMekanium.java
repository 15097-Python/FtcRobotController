package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="DirectionalMekanium")
@Disabled
public class DirectionalMekanium extends LinearOpMode {

    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor FR;

    private double V;
    private double H;
    private double R;
    @Override
    public void runOpMode() {

        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL.setDirection(DcMotor.Direction.REVERSE); //so I don't have to think about
        BL.setDirection(DcMotor.Direction.REVERSE); //inverting later
        FR.setDirection(DcMotor.Direction.FORWARD); //should generally do whenever motors
        BR.setDirection(DcMotor.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double targetdrivey = gamepad1.left_stick_y; // Forward/backward negative because it's naturally inverted
            double targetdrivex = gamepad1.left_stick_x; // side to side
            double targetturn  = gamepad1.right_stick_x/2; // Turning
            double BRmotorpower = targetdrivey+targetdrivex-targetturn;
            double BLmotorpower = targetdrivey-targetdrivex+targetturn;
            double FRmotorpower = (targetdrivey-targetdrivex)-targetturn;
            double FLmotorpower = targetdrivey+targetdrivex+targetturn;
            telemetry.addData("targetdrivex",targetdrivex);
            telemetry.addData("Back Right Motor Power is", BRmotorpower);
            telemetry.addData("Back Left Motor Power is", BLmotorpower);
            telemetry.addData("Front Right Motor Power is", FRmotorpower);
            telemetry.addData("Front Left Motor Power is", FLmotorpower);

            //assigns power to each motor based on gamepad inputs
            BR.setPower(BRmotorpower);
            BL.setPower(BLmotorpower);
            FR.setPower(FRmotorpower);
            FL.setPower(FLmotorpower);

            telemetry.addData(  "Status", "Running");
            telemetry.update();
        }
    }
}
