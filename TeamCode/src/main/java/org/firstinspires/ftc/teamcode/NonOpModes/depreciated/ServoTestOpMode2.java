package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTestOpMode2")
@Config

public class ServoTestOpMode2 extends LinearOpMode {

    private Servo DrumServo1;
    private Servo FiringPinServo;

    public static double servoOffSet = 0;
    public static int servoPosition = 0;
    public static double slot0load = 0;
    public static double slot1load = 0;
    public static double slot2load = 0;
    public static double slot0shoot = 0;
    public static double slot1shoot = 0;
    public static double slot2shoot = 0;
    public static double firingpinmax = 0.95;


    @Override
    public void runOpMode() {
        DrumServo1 = hardwareMap.get(Servo.class, "DrumServo1");
        FiringPinServo = hardwareMap.get(Servo.class, "FiringPinServo");

        double targetdrumangle = 0;
        double targetfiringpinangle = 0;
        boolean firing = false;

        DrumServo1.setPosition(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // sets the three angles

            switch (servoPosition){
                case(0):
                    targetdrumangle = slot0load;
                    break;
                case(1):
                    targetdrumangle = slot1load;
                    break;
                case(2):
                    targetdrumangle = slot2load;
                    break;
                case(3):
                    targetdrumangle = slot0shoot;
                    break;
                case(4):
                    targetdrumangle = slot1shoot;
                    break;
                case(5):
                    targetdrumangle = slot2shoot;
                    break;
            }


            DrumServo1.setPosition(targetdrumangle);
            FiringPinServo.setPosition(targetfiringpinangle);



            telemetry.addData("servo offset angle", servoOffSet);
            telemetry.addData("servoangle", targetdrumangle);
            telemetry.addData("servoangle", targetfiringpinangle);
            telemetry.update();
        }
    }
}
