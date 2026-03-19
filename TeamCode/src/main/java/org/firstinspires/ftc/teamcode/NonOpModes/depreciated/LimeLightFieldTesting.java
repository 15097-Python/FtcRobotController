package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;

import static org.firstinspires.ftc.teamcode.limelight.LimelightPosSetting.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous(name="LimeLightFieldTesting", group="limelight")


public class LimeLightFieldTesting extends LinearOpMode {

    @Override

    public void runOpMode() {

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");// INitilizes the limelights
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        Pose2d startPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        waitForStart();

        while (opModeIsActive()) { // keeps the code running so it doesn't only run once

            drive.localizer.update();
            LLResult result = limelight.getLatestResult();
            /*
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
                            telemetry.addData("bot Location", "(x" + x + ", " + y + "y)");
                        }
                    }
                }
            }
            else {
                telemetry.addLine("no robot location update");
            } */
            updateOrientation(limelight,Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            limelightposupdate(limelight);
            LLResult result2 = limelight.getLatestResult();
            Pose3D robotPoseMT2 = result2.getBotpose_MT2();
            telemetry.addData("MEGATAG2 x: ",  robotPoseMT2.getPosition().x);
            telemetry.addData("MEGATAG2 y: ", robotPoseMT2.getPosition().y);

            telemetry.update();
        }

        limelight.stop(); //stops the limelight

    } 
}

