package org.firstinspires.ftc.teamcode.limelight;

import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.getRobotCoordinates;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.modifyRobotCoordinates;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightPosSetting {
    public static void limelightposupdate(Limelight3A limelight){
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) { // checks if there is a target and if the target is an actual target

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults(); //get fiducial results basically just tells how many april tags it sees
            //List<LLResultTypes.FiducialResult>: so it makes a list at the size of the # of tags detected and has info on the id and position of the tag

            for (LLResultTypes.FiducialResult tag : tags) {
                int id = tag.getFiducialId();
                if (id == 20 || id == 24) {
                    Pose3D robotpose = tag.getRobotPoseFieldSpace();
                    if (robotpose != null) {
                        double x = robotpose.getPosition().x;
                        double y = robotpose.getPosition().y;
                        double yaw = robotpose.getOrientation().getYaw();

                        double[] currentrobotlocation = getRobotCoordinates();
                        modifyRobotCoordinates(x, y, currentrobotlocation[2], currentrobotlocation[3], currentrobotlocation[4], yaw);

                        break;
                    }
                }
            }
        }
    }
}
