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

    public static void limelightPosUpdate(Limelight3A limelight, double headingDegrees){ //this is the function used for MT2
        limelight.updateRobotOrientation(headingDegrees);
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) { // checks if there is a target and if the target is an actual target

            Pose3D robotPoseMT2 = result.getBotpose_MT2();

            if (robotPoseMT2 != null) {
                double x = robotPoseMT2.getPosition().x;
                double y = robotPoseMT2.getPosition().y;
                double yaw = robotPoseMT2.getOrientation().getYaw();
                //yaw = -yaw;  //try this later after trying the z, roll, pitch thing since I'm not actually aware of specifically the contents of what currentrobotlocation outputs
                double z = 0; //change these once I see what I have them set as in the limelight
                double roll = 0; //since these chould be constant and not changing
                double pitch = 0; //TODO fill in these constants
                double[] currentrobotlocation = getRobotCoordinates();
                modifyRobotCoordinates(x, y, z, roll, pitch, yaw);
                /*, currentrobotlocation[2], currentrobotlocation[3], currentrobotlocation[4]*/ // readd this later after verifying what it outputs
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
