package org.firstinspires.ftc.teamcode.Util;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;


public class Servo {
    static double lastAngle = 0;


    static double targetServoPower = 0;
    static int rotationCount = 0;


    final static int rotationDetectionValue = 60;
    final static double acceptableErrorDegrees = 5;




    public static void setDualServoPower(double targetAngle,AnalogInput axonEncoder, CRServo servo1, CRServo servo2){
        double voltage = axonEncoder.getVoltage();//CHATGPT SAID THIS WOULD GET VOLTAGE TAKE WITH 3LBS OF SALT code reading the analog input from the servo
        double angle = (voltage / axonEncoder.getMaxVoltage()) * 360.0;//Note I stole this from some random person who was doing this in 2010


        double angleChange = angle - lastAngle;
        if(angleChange < -rotationDetectionValue) rotationCount++;
        if(angleChange > rotationDetectionValue) rotationCount--;
        lastAngle = angle;

        servo1 = hardwareMap.get(CRServo.class, "DrumServo1");
        servo2 = hardwareMap.get(CRServo.class, "DrumServo2");

        double trueAngle = rotationCount * 360 + angle;


        //Everthing below this line is only used for the laziest possible way of getting servos to the right angle. Unless this magicaly works perfectly we should probably utilize PID for this.
        if(targetAngle - trueAngle > acceptableErrorDegrees) targetServoPower = 1;
        if(targetAngle - trueAngle < acceptableErrorDegrees) targetServoPower = -1;
        servo1.setPower(targetServoPower);
        //servo2.setPower(targetServoPower);


        telemetry.addData("trueAngle",trueAngle);
        telemetry.addData("rotationcount",rotationCount);
        telemetry.addData("angle",angle);
        telemetry.addData("angleChange",angleChange);
        telemetry.addData("lastAngle",lastAngle);


    }
}

