package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PathBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="RoadRunnerTesting")
public class RoadRunnerTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(  hardwareMap,  startPose);



        // your drive class

        waitForStart();
        if (isStopRequested()) return;

        Action moveOneWay = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(12)
                .build();

        Actions.runBlocking(moveOneWay);

        Pose2d newpose = drive.localizer.getPose();

        Double seocndx =  newpose.position.x;
        Double secondy = newpose.position.y;
        Double secondtheta = newpose.heading.toDouble();


        Action movethesecond = drive.actionBuilder(new Pose2d(seocndx,secondy,secondtheta))
                .splineTo(new Vector2d(5,-6),Math.toRadians(180))
                .build();

        Actions.runBlocking(movethesecond);

        /*
        Action path = new PathBuilder(new Pose2d(0, 0, 0))
                .splineTo(new Pose2d(15, 15, 0))
                .lineTo(new Vector2d(30, 15))
                .build();
        */


    }
}
