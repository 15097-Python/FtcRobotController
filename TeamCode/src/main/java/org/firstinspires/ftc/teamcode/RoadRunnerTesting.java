package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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

        // ----- First trajectory: move to (6, 0) -----
        com.acmerobotics.roadrunner.Action traj1 =
                drive.actionBuilder(startPose)
                        .lineTo(new Vector2d(6, 0))
                        .build();

// Run first trajectory
        Actions.runBlocking(traj1);

// ----- Second trajectory: move to (-3, 4) -----


// The second trajectory should start from the robot’s current pose estimate
        Pose2d secondStart = drive.localizer.getPose();

        com.acmerobotics.roadrunner.Action traj2 =
                drive.actionBuilder(secondStart)
                        .lineTo(new Vector2d(-3, 4))
                        .build();

// Run second trajectory
        Actions.runBlocking(traj2);



    }
}
