package org.firstinspires.ftc.teamcode.limelight;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;


public class LimelightPosSetting {

    public static void limelightPosUpdate(Limelight3A limelight, MecanumDrive drive, double headingDegrees) { //for MT2
        limelight.updateRobotOrientation(headingDegrees);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D robotPoseMT2 = result.getBotpose_MT2();

            if (robotPoseMT2 != null) {
                // Convert meters to inches
                double x = robotPoseMT2.getPosition().x * 39.37;
                double y = robotPoseMT2.getPosition().y * 39.37;

                // Convert CW degrees to CCW radians for RoadRunner
                double yawDegrees = robotPoseMT2.getOrientation().getYaw();
                double yawRadians = Math.toRadians(-yawDegrees);

                drive.localizer.setPose(new Pose2d(new Vector2d(x, y), yawRadians));
            }
        }
    }

    public static void roadrunnerupdatevialimelight(Limelight3A limelight,MecanumDrive drive){ //this is for MT1
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) { // checks if there is a target and if the target is an actual target

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults(); //get fiducial results basically just tells how many april tags it sees
            //List<LLResultTypes.FiducialResult>: so it makes a list at the size of the # of tags detected and has info on the id and position of the tag

            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (id == 20 || id == 24) {
                    Pose3D robotpose = tag.getRobotPoseFieldSpace();
                    if (robotpose != null) {
                        double ta = tag.getTargetArea();

                        double x = 39.37 * robotpose.getPosition().x;
                        double y = 39.37 * robotpose.getPosition().y;
                        double yaw = Math.toRadians(robotpose.getOrientation().getYaw());

                        drive.localizer.setPose(new Pose2d(new Vector2d(x,y),yaw));

                        break;
                    }
                }
            }
        }
    }
}
