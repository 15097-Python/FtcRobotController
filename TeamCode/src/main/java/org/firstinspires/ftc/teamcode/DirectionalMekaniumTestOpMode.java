package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DirectionalMekaniumTestOpMode")

public class DirectionalMekaniumTestOpMode extends LinearOpMode {

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


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            V = .5;
            H = 0;
            R = 0;

            BR.setPower(V+H-R);
            BL.setPower(V-H+R);
            FR.setPower(-((V-H)-R));
            FL.setPower(-(V+H+R));

            telemetry.addData(  "Status", "Running");
            telemetry.update();
        }
    }
}
