package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DirectionalMekaniumTestOpMode")

public class DirectionalMekaniumTestOpModeCrosby extends LinearOpMode {

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
            double targetDriveY = -gamepad1.left_stick_y; // Forward/backward negative because it's naturally inverted
            double targetDriveX = gamepad1.left_stick_x; // side to side
            double targetTurn  = gamepad1.right_stick_x; // Turning
            double BRMotorPower = targetDriveY+targetDriveX-targetTurn;
            double BLMotorPower = targetDriveY-targetDriveX+targetTurn;
            double FRMotorPower = -((targetDriveY-targetDriveX)-targetTurn);
            double FLMotorPower = -(targetDriveY+targetDriveX+targetTurn);
            telemetry.addData("Back Right Motor Power is", BRMotorPower);
            telemetry.addData("Back Left Motor Power is", BLMotorPower);
            telemetry.addData("Front Right Motor Power is", FRMotorPower);
            telemetry.addData("Front Left Motor Power is", FLMotorPower);

            //assigns power to each motor based on gamepad inputs
            BR.setPower(targetDriveY+targetDriveX-targetTurn);
            BL.setPower(targetDriveY-targetDriveX+targetTurn);
            FR.setPower(targetDriveY-targetDriveX-targetTurn);
            FL.setPower(targetDriveY+targetDriveX+targetTurn);

            telemetry.addData(  "Status", "Running");
            telemetry.update();
        }
    }
}
