package org.firstinspires.ftc.teamcode.limelight;

import static org.firstinspires.ftc.teamcode.Util.RobotPosition.getRobotCoordinates;
import static org.firstinspires.ftc.teamcode.Util.RobotPosition.modifyRobotCoordinates;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
                double yaw = Math.toRadians(robotPoseMT2.getOrientation().getYaw());

                double[] currentrobotlocation = getRobotCoordinates();
                modifyRobotCoordinates(x, y, currentrobotlocation[2], currentrobotlocation[3], currentrobotlocation[4], yaw);
            }
        }
    }
}
