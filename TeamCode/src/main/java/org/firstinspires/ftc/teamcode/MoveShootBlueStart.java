package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.TeamColorRED;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robottargetyaw;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.robotyaw;
import static org.firstinspires.ftc.teamcode.launcher.AutonoumusAutoLaunch.autoLaunchWithAim;
import static org.firstinspires.ftc.teamcode.limelight.LimelightPosSetting.limelightposupdate;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NonOpModes.PID.PIDCrosby;

@Autonomous(name="BlueShootGoalStart")
@Config

public class MoveShootBlueStart extends LinearOpMode {
    public static long rotatetime = 320;
    public static long drivetime = 600;

    public static double powermod = 2;
    public static double targetangle = 135;
    public static double frredom = .1;


    private GoBildaPinpointDriver odomhub;

    @Override
    public void runOpMode() {
        double targetturn = 0;
        TeamColorRED = false;

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");// INitilizes the limelights
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();


        odomhub = hardwareMap.get(GoBildaPinpointDriver.class, "odomhub");
        odomhub.initialize();
        odomhub.resetPosAndIMU();

        ElapsedTime timer = new ElapsedTime();
        double[] loadedBalls = {1,2,3};

        Servo drumServo = hardwareMap.get(Servo.class, "DrumServo");
        Servo firingPinServo = hardwareMap.get(Servo.class, "FiringPinServo");



        DcMotorEx LauncherFL = hardwareMap.get(DcMotorEx.class, "LauncherFL");

        DcMotor FL = hardwareMap.get(DcMotor.class, "FL"); // local hardware mapping
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.FORWARD); //so I don't have to think about
        BL.setDirection(DcMotor.Direction.FORWARD); //inverting later
        FR.setDirection(DcMotor.Direction.REVERSE); //should generally do whenever motors
        BR.setDirection(DcMotor.Direction.REVERSE);

        double[] drumLocations = {0.1, 0.42, 0.76};// should probably make the drumb slots into objects
        int i = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();/////////////////////////////////////////////



            FL.setPower(1); //tells the motors how fast to go
            BL.setPower(1);
            FR.setPower(-1);
            BR.setPower(-1);
            sleep(rotatetime);

            FL.setPower(1); //tells the motors how fast to go
            BL.setPower(-1);
            FR.setPower(-1);
            BR.setPower(1);
            sleep(drivetime);
            FL.setPower(-1); //tells the motors how fast to go
            BL.setPower(-1);
            FR.setPower(1);
            BR.setPower(1);
            /*sleep(rotatetime/2);
            FL.setPower(0); //tells the motors how fast to go
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            firingPinServo.setPosition(.98);*/


            limelightposupdate(limelight);

            autoLaunchWithAim(LauncherFL,drumServo,firingPinServo,1,loadedBalls);
            //yaw align loop
            //robottargetyaw = targetangle;
            while (opModeIsActive()) {
                limelightposupdate(limelight);
                //double headingoffset = odomhub.getHeading(AngleUnit.DEGREES) - robotyaw;
                telemetry.addData("targetyaw", robottargetyaw);
                telemetry.addData("robotyaw", robotyaw);
                telemetry.update();


                double timeBetweenLoops = timer.milliseconds();  // miliseconds since last reset
                timer.reset();
                targetturn = PIDCrosby.settingMotorPIDPowerYaw(timeBetweenLoops);

                if (targetturn > 1) targetturn = 1;
                if (targetturn < -1) targetturn = -1;
                double targetdrivey = 0;
                double targetdrivex = 0;


                double BRmotorpower = targetdrivey + targetdrivex - targetturn;
                double BLmotorpower = targetdrivey - targetdrivex + targetturn;
                double FRmotorpower = (targetdrivey - targetdrivex) - targetturn;
                double FLmotorpower = targetdrivey + targetdrivex + targetturn;

                BR.setPower(BRmotorpower / powermod);//set the motors power
                BL.setPower(BLmotorpower / powermod);
                FR.setPower(FRmotorpower / powermod);
                FL.setPower(FLmotorpower / powermod);

                //telemetry.addLine("got the motors power set out of the loop");
                //telemetry.update();
                //(robotyaw - frredom) > robottargetyaw && (robotyaw + frredom) < robottargetyaw
                if (Math.abs(targetturn)    < frredom) {
                    //LauncherFL.setVelocity(autoLaunch(LauncherFL, DrumServo, FiringPinServo, 1, drumBallColors),AngleUnit.RADIANS);
                    FL.setPower(0); //tells the motors how fast to go
                    BL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    sleep(1000);
                    while (i < 3) {//slot finding loop
                        telemetry.addData("shooting", i);
                        telemetry.addData("motorpower", targetturn);
                        telemetry.update();

                        drumServo.setPosition(drumLocations[i]);
                        LauncherFL.setVelocity(autoLaunchWithAim(LauncherFL, drumServo, firingPinServo, 1, loadedBalls), AngleUnit.RADIANS);
                        sleep(500);
                        firingPinServo.setPosition(.98 - .32);
                        sleep(200);
                        firingPinServo.setPosition(.98);
                        sleep(100);


                        i++;
                    }

                }
                if(i >= 3) {
                    FL.setPower(-1); //tells the motors how fast to go
                    BL.setPower(-1);
                    FR.setPower(-1);
                    BR.setPower(-1);
                    sleep(200);
                    break;
                }


            }


    }

}
