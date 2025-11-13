/*package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="ServoTestOpMode")

public class ServoTestOpMode extends LinearOpMode {

    private Servo servo;


    @Override
    public void runOpMode() {

    Servo DrumServo = hardwareMap.get(Servo.class, "servo");
    Servo FiringPinServo = hardwareMap.get(Servo.class, "servo");
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
            if (FiringPinServo.getPosition <= 0.01) DrumServo.setPosition(targetdrumangle)// this likely doesn't actually work to prevent errors and we need to use the voltage retuned from the servo

            if (DrumServo.getPosition >= (targetdrumangle - .01) && DrumServo.getPosition <= (targetdrumangle + .01)){
                FiringPinServo.setPosition(1);
            }
            
            telemetry.addData("servoangle", servoangle);
            telemetry.update();
        }
    }
}
*/