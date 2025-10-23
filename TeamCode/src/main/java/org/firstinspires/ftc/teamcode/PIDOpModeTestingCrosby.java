package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NonOpModes.PID.PIDCrosby;
@TeleOp(name="DirectionalMekaniumTestOpModeCrosby")

public class PIDOpModeTestingCrosby extends LinearOpMode {

    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor FR;

    ElapsedTime timer = new ElapsedTime();
    double xTargetCoordinate = 0;
    double xCurrentCoordinate = 0;
    double yTargetCoordinate = 0;
    double yCurrentCoordinate = 0;
    double turnTargetRadian = 0;
    double turnCurrentRadian = 0;


    private double V;
    private double H;
    static double timeBetweenLoops = 0;
    private double R;
    @Override
    public void runOpMode() {
       double timeBetweenLoops = 0;  // seconds since last reset

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
            timeBetweenLoops = timer.seconds();  // seconds since last reset
            timer.reset();
            double targetdrivey = PIDCrosby.settingMotorPIDPower(xTargetCoordinate,xCurrentCoordinate,null,null,null,null,timeBetweenLoops)[0];
            double targetdrivex = PIDCrosby.settingMotorPIDPower(null,null,yTargetCoordinate,yCurrentCoordinate,null,null,timeBetweenLoops)[1];
            double targetturn = PIDCrosby.settingMotorPIDPower(null,null,null,null,turnTargetRadian,turnCurrentRadian,timeBetweenLoops)[2];
            double BRmotorpower = targetdrivey+targetdrivex-targetturn;
            double BLmotorpower = targetdrivey-targetdrivex+targetturn;
            double FRmotorpower = -((targetdrivey-targetdrivex)-targetturn);
            double FLmotorpower = -(targetdrivey+targetdrivex+targetturn);
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
