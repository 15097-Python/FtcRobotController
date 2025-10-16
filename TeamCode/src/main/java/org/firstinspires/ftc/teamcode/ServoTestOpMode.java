package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="ServoTestOpMode")

public class ServoTestOpMode extends LinearOpMode {

    private Servo servo;


    @Override
    public void runOpMode() {

    servo = hardwareMap.get(Servo.class, "servo");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            servo.setPosition(.75);

            telemetry.addData(  "Status", "Running");
            telemetry.update();
        }
    }
}
