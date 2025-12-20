package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.Enum.Balls.green;
import static org.firstinspires.ftc.teamcode.Util.Enum.Balls.purple;
import static org.firstinspires.ftc.teamcode.Util.Enum.Balls.unknown;
import static org.firstinspires.ftc.teamcode.Util.RRSplineToLaunchPos.splineLaunchPos;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.FIELD_HALF;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.mtoin;
import static org.firstinspires.ftc.teamcode.Util.constants.RobotStats.firingpinfiringposition;
import static org.firstinspires.ftc.teamcode.Util.constants.RobotStats.firingpinnullposition;
import static org.firstinspires.ftc.teamcode.launcher.AutoFirePower.autoLaunch;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.Enum.Balls;

@Autonomous(name="FullRRtestautoTurning")

public class AutoWithRoadRunnerStrafe extends LinearOpMode {
    private Servo DrumServo;
    private Servo FiringPinServo;
    private DcMotorEx LauncherFL;

    @Override
    public void runOpMode() {

        DrumServo = hardwareMap.get(Servo.class, "DrumServo");
        FiringPinServo = hardwareMap.get(Servo.class, "FiringPinServo");

        double[] firingpositions = {.1,.42,.76};
        double motortargetspeedradians = 0;
        Balls[] drumBallColors = {purple,green,purple};
        Balls[] targetballcolors = {green,purple,purple};

        double firingpositionstarget;
        double targetdrumangle;

        // y = x
        // x = -y
        Pose2d startPose = new Pose2d(-62, 37.5, 0);
        MecanumDrive drive = new MecanumDrive(  hardwareMap,  startPose);

        waitForStart();

        FiringPinServo.setPosition(firingpinnullposition);

        telemetry.addLine("moving to launch zone");
        telemetry.update();

        splineLaunchPos(drive,startPose,-170);
        startPose = drive.localizer.getPose();

        telemetry.addLine("moving to launch zone");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();

        //limelight get pattern

        splineLaunchPos(drive,startPose,140);
        startPose = drive.localizer.getPose();

        telemetry.addLine("moving to first load");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();

        motortargetspeedradians = autoLaunch();
        LauncherFL.setVelocity(motortargetspeedradians, AngleUnit.RADIANS);


        for(int i = 0; i <= 2; i++){
            for(int j = 0; j<= 2; j++) {
                if (drumBallColors[j] == targetballcolors[i]){
                    drumBallColors[j] = unknown;
                    targetdrumangle = firingpositions[j];
                    DrumServo.setPosition(targetdrumangle);
                    sleep(400);
                    FiringPinServo.setPosition(firingpinfiringposition);
                    sleep(200);
                    FiringPinServo.setPosition(firingpinnullposition);
                    sleep(200);
                }
            }

        }



        //launch with pattern

        Action moveToLoadingOne = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(-12,22.5),Math.toRadians(0))
                .build();
        Actions.runBlocking(moveToLoadingOne);
        startPose = drive.localizer.getPose();

        telemetry.addLine("doing first load");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();

        //intake

        Action pickUpLoadOne = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(-12,58.5),Math.toRadians(0))
                .build();
        Actions.runBlocking(pickUpLoadOne);
        startPose = drive.localizer.getPose();

        telemetry.addLine("moving to launch zone");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();

        splineLaunchPos(drive,startPose,0);
        startPose = drive.localizer.getPose();
        //luanch

        telemetry.addLine("moving to second load ");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();
        Action moveToLoadingTwo = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(12,22.5),Math.toRadians(0))
                .build();
        Actions.runBlocking(moveToLoadingTwo);
        startPose = drive.localizer.getPose();

        telemetry.addLine("second load");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();
        //intake

        Action pickUpLoadTwo = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(12,58.5),Math.toRadians(0))
                .build();
        Actions.runBlocking(pickUpLoadTwo);
        startPose = drive.localizer.getPose();

        telemetry.addLine("moving to launch zone");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();

        splineLaunchPos(drive,startPose,0);
        startPose = drive.localizer.getPose();

        telemetry.addLine("moving to load three ");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();

        //launch

        Action moveToLoadingThree = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(30,22.5),Math.toRadians(0))
                .build();
        Actions.runBlocking(moveToLoadingThree);
        startPose = drive.localizer.getPose();

        telemetry.addLine("doing third load");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.update();

        //intake

        Action pickUpLoadThree = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(30,58.5),Math.toRadians(0))
                .build();
        Actions.runBlocking(pickUpLoadThree);
        startPose = drive.localizer.getPose();

        telemetry.addLine("moving to launch zone");
        telemetry.addData("x" , startPose.position.x);
        telemetry.addData("y", startPose.position.y);
        telemetry.addLine("end");
        telemetry.update();

        splineLaunchPos(drive,startPose,0);
    }
}
