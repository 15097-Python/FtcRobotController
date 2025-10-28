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
    double servoangle = 0;
    int incramentor = 0;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            incramentor++;
            servo.setPosition(servoangle);

            if (servoangle > 1){
                servoangle = 0;
            }
            servoangle += .33;
            sleep(1000);
            telemetry.addData(  "servoangle", servoangle);
            telemetry.addData("coutning", incramentor);
            telemetry.update();
        }
    }
}
