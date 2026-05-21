package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Util.Servo.setDualServoPower;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="servoTest")
public class ServoTest extends LinearOpMode {


    CRServo drumServo1 = hardwareMap.get(CRServo.class, "DrumServo1");
    CRServo drumServo2 = hardwareMap.get(CRServo.class, "DrumServo2");
    //Servo drumServo = hardwareMap.get(Servo.class, "DrumServo");
    AnalogInput servoValue = hardwareMap.get(AnalogInput.class, "ServoValue");




    @Override
    public void runOpMode() {




        double targetAngle = 0;
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpad_up)targetAngle+=5;
            if(gamepad1.dpad_down)targetAngle-=5;
            if(gamepad1.a)targetAngle = 0;
             setDualServoPower(targetAngle,servoValue,drumServo1,drumServo2);
            telemetry.addData("targetAngle",targetAngle);
            telemetry.update();
        }
    }
}



