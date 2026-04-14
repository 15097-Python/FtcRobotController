package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.RobotPosition.TeamColorRED;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.shoottargetx;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.shoottargetyblue;
import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.shoottargetyred;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

            double arctanintermediatex = shoottargetx*39.3701-drive.localizer.getPose().position.y;
            double arctanintermediatey;
            double usedy;

            //converts inches to meters
            telemetry.addData("robotx", drive.localizer.getPose().position.y);
            telemetry.addData("roboty",drive.localizer.getPose().position.x);
            if (TeamColorRED) usedy = shoottargetyred * 39.3701;
            else usedy = shoottargetyblue * 39.3701;
            arctanintermediatey = usedy + drive.localizer.getPose().position.x;
            double robotautoaimtargetangle = atan2(arctanintermediatex, arctanintermediatey);
            telemetry.addData("rawangle",robotautoaimtargetangle);
            //if(robotautoaimtargetangle<0) robotautoaimtargetangle= Math.PI + robotautoaimtargetangle;
            Action movetoloadingone = drive.actionBuilder(drive.localizer.getPose())
                    .turnTo(robotautoaimtargetangle)
                    .build();
            if(gamepad1.b) {
                Actions.runBlocking(movetoloadingone);
            }


            telemetry.addData("robotr", drive.localizer.getPose().heading.toDouble());
            telemetry.addData("robat aurot aim target angle", robotautoaimtargetangle);
            telemetry.update();




        }
    }
}
