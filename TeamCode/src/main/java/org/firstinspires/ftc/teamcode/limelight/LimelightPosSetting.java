package org.firstinspires.ftc.teamcode.limelight;

import static org.firstinspires.ftc.teamcode.Util.RobotPosition.getRobotCoordinates;
import static org.firstinspires.ftc.teamcode.Util.RobotPosition.modifyRobotCoordinates;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;


//+X is forward, +Y is left
public class LimelightPosSetting {
    //need to call this at loops because MT2 needs the gyro heading
    public static void updateOrientation(Limelight3A limelight, double headingDegrees) {
        limelight.updateRobotOrientation(headingDegrees);
    } //this needs to be degrees as occording to MT2

    public static void limelightposupdate(Limelight3A limelight){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) { // checks if there is a target and if the target is an actual target

            Pose3D robotPoseMT2 = result.getBotpose_MT2();

            if (robotPoseMT2 != null) {
                double x = robotPoseMT2.getPosition().x;
                double y = robotPoseMT2.getPosition().y;
                double yaw = robotPoseMT2.getOrientation().getYaw();

                double[] currentrobotlocation = getRobotCoordinates();
                modifyRobotCoordinates(x, y, currentrobotlocation[2], currentrobotlocation[3], currentrobotlocation[4], yaw);
            }
        }
    }
    public static void roadrunnerupdatevialimelight(Limelight3A limelight,MecanumDrive drive){
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
