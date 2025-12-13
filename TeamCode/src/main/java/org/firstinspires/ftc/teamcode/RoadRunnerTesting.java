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


// Run first trajectory


// ----- Second trajectory: move to (-3, 4) -----


// The second trajectory should start from the robot’s current pose estimate
        Pose2d secondStart = drive.localizer.getPose();



// Run second trajectory




    }
}
