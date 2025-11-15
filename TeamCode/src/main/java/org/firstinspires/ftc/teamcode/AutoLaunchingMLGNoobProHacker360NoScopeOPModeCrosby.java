package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.getRobotCoordinates;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.modifyRobotCoordinates;
import static org.firstinspires.ftc.teamcode.launcher.AutoLaunchingMLGNoobProHacker360NoScopeAbstract.autoLaunch;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.positioning.odometry.FieldOrientedDriving;
import org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby;

import java.util.List;

@TeleOp(name="autholoaunchermode")

public class AutoLaunchingMLGNoobProHacker360NoScopeOPModeCrosby extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private Servo DrumServo;
    private Servo FiringPinServo;
    private GoBildaPinpointDriver odomhub;
    private DcMotorEx Scooper;
    private DcMotor BR;
    private DcMotor BL;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotorEx LauncherFL;


    @Override
    public void runOpMode() {
        double lasttime = timer.milliseconds();

        double[] drumBallColors = {0, 0, 0};
        double targetdrumangle = 0;
        double targetfiringpinangle = 0;
        boolean firing = false;

        double motortargetspeedradians = 0;
        double currentleftmotorvelocity = 0;
        double currentrightmotorvelocity = 0;

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");// INitilizes the limelights
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        odomhub = hardwareMap.get(GoBildaPinpointDriver.class,"odomhub");

        

        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL.setDirection(DcMotor.Direction.REVERSE); //so I don't have to think about
        BL.setDirection(DcMotor.Direction.REVERSE); //inverting later
        FR.setDirection(DcMotor.Direction.FORWARD); //should generally do whenever motors
        BR.setDirection(DcMotor.Direction.FORWARD);

        LauncherFL = hardwareMap.get(DcMotorEx.class, "LauncherFL");
        Scooper = hardwareMap.get(DcMotorEx.class,"Scooper");

        //DcMotorEx LauncherFR = hardwareMap.get(DcMotorEx.class, "LauncherFR");
        

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        //zeros the encoders and sets the run using encoder mode
        //VariablePowerLauncherAbstract.initializeLauncher(LauncherFL,LauncherFR);
        LauncherFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LauncherFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Scooper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Scooper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odomhub.initialize();
        odomhub.resetPosAndIMU();   // resets encoders and IMU

        FiringPinServo.setPosition(0);
        

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double leftstickinputy = gamepad1.left_stick_y; // Forward/backward negative because it's naturally inverted
            double leftstickinputx = gamepad1.left_stick_x; // side to side
            double targetturn  = gamepad1.right_stick_x/2; // Turning

            double [] currentrobotlocation = getRobotCoordinates();

            /* Hopefully deprecated
            //increments the target speed with up and right while decrementing it with left and down
            motortargetspeedradians += gamepad1.dpadUpWasPressed() ? 1 : 0;
            motortargetspeedradians -= gamepad1.dpadDownWasPressed() ? 1 : 0;
            motortargetspeedradians += gamepad1.dpadRightWasPressed() ? .1 : 0;
            motortargetspeedradians -= gamepad1.dpadLeftWasPressed() ? .1 : 0;*/

            double currentrelativeheading = odomhub.getHeading(AngleUnit.RADIANS);


            //Calls FieldOrientedDriving function and sets motor power
            double[] motorpowerarray = FieldOrientedDriving.fieldOrientedMath(leftstickinputy, leftstickinputx, targetturn, currentrelativeheading);

            double BRmotorpower = motorpowerarray[0];
            double BLmotorpower = motorpowerarray[1];
            double FRmotorpower = motorpowerarray[2];
            double FLmotorpower = motorpowerarray[3];
            // 0 is empty, 1 is green ball, 2 is purple ball

            //Limelight stuff
            if (gamepad1.right_trigger >= 0.1){
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()){ // checks if there is a target and if the target is an actual target

                    List<LLResultTypes.FiducialResult> tags = result.getFiducialResults(); //get fiducial results basically just tells how many april tags it sees
                    //List<LLResultTypes.FiducialResult>: so it makes a list at the size of the # of tags detected and has info on the id and position of the tag

                    for (LLResultTypes.FiducialResult tag : tags) {
                        int id = tag.getFiducialId();
                        if (id == 20 || id == 24){
                            Pose3D robotpose = tag.getRobotPoseFieldSpace();
                            if (robotpose != null) {
                                double x = robotpose.getPosition().x;
                                double y = robotpose.getPosition().y;

                                modifyRobotCoordinates(x,y,currentrobotlocation[2],currentrobotlocation[3],currentrobotlocation[4],currentrobotlocation[5]);
                                break;
                            }
                        }
                    }
                }

            }

            motortargetspeedradians = autoLaunch(LauncherFL,DrumServo,FiringPinServo,1, drumBallColors);

            // sets the velocity of the motors
            LauncherFL.setVelocity(motortargetspeedradians);

            currentleftmotorvelocity = LauncherFL.getVelocity(AngleUnit.RADIANS);
            //currentrightmotorvelocity = LauncherFR.getVelocity(AngleUnit.RADIANS);
            double rawrightmotorvelocity = LauncherFL.getVelocity();



            

            /*if (!firing){// This code is for when we get the voltages for now the drivers
                targetfiringpinangle = 0; have to be carefull
                targetdrumangle = gamepad1.x ? .2 :
                                  gamepad1.y ? .5 :
                                  gamepad1.b ? .8 :
                                  0;
            }

            //waits untill the firingpinservo is at its fully retracted point
            if (FiringPinServo.getPosition() >= -0.01 && FiringPinServo.getPosition() <= 0.01){
                DrumServo.setPosition(targetdrumangle);
                firing = true;
            } 
            //ensures the drumbservo is at its spot before moving the firing pin servo
            // this likely doesn't actually work to prevent errors and we need to use the voltage retuned from the servo
            if (DrumServo.getPosition() >= (targetdrumangle - .01) && DrumServo.getPosition() <= (targetdrumangle + .01)){
                targetfiringpinangle = 1;
                firing = false;
            }*/
            if (gamepad1.a) {
                targetfiringpinangle = 1;
            } else {
                targetfiringpinangle = 0;// these values are all placeholders
                targetdrumangle = gamepad1.x ? .2 :
                                  gamepad1.y ? .5 :
                                  gamepad1.b ? .8 :
                                  0;
            }
            DrumServo.setPosition(targetdrumangle);
            FiringPinServo.setPosition(targetfiringpinangle);

            //assigns power to each motor based on gamepad inputs
            BR.setPower(BRmotorpower);
            BL.setPower(BLmotorpower);
            FR.setPower(FRmotorpower);
            FL.setPower(FLmotorpower);

            if (gamepad1.left_bumper) Scooper.setVelocity(999,AngleUnit.RADIANS);
            if (gamepad1.right_bumper) Scooper.setVelocity(-999,AngleUnit.RADIANS);

            odomhub.update();

            telemetry.addLine("All Speeds are in Jacks Per Second");
            telemetry.addData("Motors' Target Rate of Rotation ", motortargetspeedradians);
            telemetry.addData("Left Motor Actual Rate of Rotation", currentleftmotorvelocity);
            //telemetry.addData("Right Motor Actual Rate of Rotation", currentrightmotorvelocity);
            //telemetry.addData("rightmotorraw", rawrightmotorvelocity);
            telemetry.addData("Left Motor difference in Rate of Rotation", motortargetspeedradians-currentleftmotorvelocity);
            //telemetry.addData("Right Motor difference in Rate of Rotation", motortargetspeedradians+currentrightmotorvelocity);
            //telemetry.addData("Left Motor Speed at Wheel Surface meters per second",currentleftmotorvelocity*launcherwheelradiusm);
            //telemetry.addData("Right Motor Speed at Wheel Surface meters per second",currentrightmotorvelocity*launcherwheelradiusm);
            telemetry.addData("drim target servoangle", targetdrumangle);
            telemetry.addData("firingpin target servoangle", targetfiringpinangle);
            telemetry.addData("rotation perceived",currentrelativeheading);
            telemetry.update();
        }
    }
}
