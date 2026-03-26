package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.NonOpModes.colorsensing.ColorSensingFunctions.colorDetection;
import static org.firstinspires.ftc.teamcode.Util.Enum.Balls.green;
import static org.firstinspires.ftc.teamcode.Util.Enum.Balls.purple;
import static org.firstinspires.ftc.teamcode.Util.Enum.Balls.unknown;
import static org.firstinspires.ftc.teamcode.Util.RRSplineToLaunchPos.splineLaunchPos;
import static org.firstinspires.ftc.teamcode.Util.RobotPosition.TeamColorRED;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.shoottargetx;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.shoottargetyblue;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.shoottargetyred;
import static org.firstinspires.ftc.teamcode.Util.constants.RobotStats.firingpinfiringposition;
import static org.firstinspires.ftc.teamcode.Util.constants.RobotStats.firingpinnullposition;
import static org.firstinspires.ftc.teamcode.limelight.LimelightMotifSetting.limelightMotifSet;

import static java.lang.Math.atan2;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.Enum.Balls;
import org.firstinspires.ftc.teamcode.Util.RobotPosition;

@Autonomous(name="autoaimtesting")
@Config

public class AutoAimTargeting extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    private DcMotorEx Scooper;
    private Servo DrumServo;
    private Servo FiringPinServo;
    private DcMotorEx LauncherFL;

    public static double xdistance = 12;
    public static double ydistance = 12;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");// INitilizes the limelights
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        Scooper = hardwareMap.get(DcMotorEx.class, "Scooper");

        DrumServo = hardwareMap.get(Servo.class, "DrumServo");
        FiringPinServo = hardwareMap.get(Servo.class, "FiringPinServo");

        LauncherFL = hardwareMap.get(DcMotorEx.class, "LauncherFL");


        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(  hardwareMap,  startPose);

        waitForStart();////////////////////////////////////////////////////

        while(opModeIsActive()) {
            drive.localizer.update();


            RobotPosition.modifyRobotCoordinates(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, 0.0,0.0,0.0,0.0);
            double[] robotcoordinates = RobotPosition.getRobotCoordinates();

            double arctanintermediatex = shoottargetx-robotcoordinates[0]/39.3701;
            double arctanintermediatey;
            double usedy;

            telemetry.addData("robotx", robotcoordinates[0]/39.3701);
            telemetry.addData("roboty", robotcoordinates[1]/39.3701);
            if (TeamColorRED) usedy = shoottargetyred;
            else usedy = shoottargetyblue;
            arctanintermediatey = usedy - robotcoordinates[1]/39.3701;
            double robotautoaimtargetangle = atan2(arctanintermediatey, arctanintermediatex);
            telemetry.addData("rawangle",robotautoaimtargetangle);
            //if(robotautoaimtargetangle<0) robotautoaimtargetangle= Math.PI + robotautoaimtargetangle;
            Action movetoloadingone = drive.actionBuilder(drive.localizer.getPose())
                    .turnTo(robotautoaimtargetangle)
                    .build();
            if(gamepad1.b) {
                Actions.runBlocking(movetoloadingone);
            }




            robotcoordinates = RobotPosition.getRobotCoordinates();

            telemetry.addData("robotr", drive.localizer.getPose().heading.toDouble());
            telemetry.addData("robat aurot aim target angle", robotautoaimtargetangle);
            telemetry.update();




        }
    }
}
