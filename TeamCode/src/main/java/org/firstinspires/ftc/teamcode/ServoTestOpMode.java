package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="ServoTestOpMode")

public class ServoTestOpMode extends LinearOpMode {

    private Servo servo;


    @Override
    public void runOpMode() {

    drumservo = hardwareMap.get(Servo.class, "servo");
    firingpinservo = hardwareMap.get(SErvo.class, "servo");
    double targetdrumangle = 0;
    double targetfiringpinangle = 0;
    boolean firing = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // sets the three angles
            if (!firing){
                targetdrumangle = gamepad1.b() ? .2 : 0;
                targetdrumangle = gamepad1.b() ? .5 : 0;
                targetdrumangle = gamepad1.b() ? .8 : 0;
            }

            // prevents the firing pin and drum from getting stuck on eachother
            if (firingpinservo.getPosition <= 0.01) drumservo.setPosition(targetdrumangle)// this likely doesn't actually work to prevent errors and we need to use the voltage retuned from the servo

            if (drumservo.getPosition >= (targetdrumangle - .01) && drumservo.getPosition <= (targetdrumangle + .01)){
                firingpinservo.setPosition(1);
            }
            
            telemetry.addData("servoangle", servoangle);
            telemetry.update();
        }
    }
}
