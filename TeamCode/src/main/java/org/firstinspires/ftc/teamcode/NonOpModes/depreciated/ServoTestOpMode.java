package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTestOpMode")
@Config

public class ServoTestOpMode extends LinearOpMode {

    private Servo DrumServo1;
    private Servo DrumServo2;
    private Servo FiringPinServo;

    public static double servoOffSet = 0;
    public static double firingpinmax = 0.95;


    @Override
    public void runOpMode() {
        DrumServo1 = hardwareMap.get(Servo.class, "DrumServo1");
        DrumServo2 = hardwareMap.get(Servo.class, "DrumServo2");
        FiringPinServo = hardwareMap.get(Servo.class, "FiringPinServo");

        double targetdrumangle = 0;
        double targetfiringpinangle = 0;
        boolean firing = false;

        DrumServo1.setPosition(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            FiringPinServo.setPosition(0 );
            // sets the three angles
            if (gamepad2.a) {
                targetfiringpinangle = firingpinmax;
            } else {
                targetfiringpinangle = .98;// these values are all placeholders
                targetdrumangle = gamepad2.x ? servoOffSet+.345 ://Firing 0
                                  gamepad2.y ? servoOffSet+.01 :
                                  gamepad2.b ? servoOffSet+.6785 :
                                  gamepad1.x ? servoOffSet+.0975 ://load 0
                                  gamepad1.y ? servoOffSet+.26 :
                                  gamepad1.b ? servoOffSet+.43 :
                                  targetdrumangle;

                //.27 - .42   0  -   1
                //.6 - .76    1   -   2
                //.92 - .9    2   -    0
            }
            DrumServo1.setPosition(targetdrumangle);
            DrumServo2.setPosition(targetdrumangle);


            telemetry.addData("servo offset angle", servoOffSet);
            telemetry.addData("servoangle", targetdrumangle);
            telemetry.addData("servoangle", targetfiringpinangle);
            telemetry.update();
        }
    }
}
