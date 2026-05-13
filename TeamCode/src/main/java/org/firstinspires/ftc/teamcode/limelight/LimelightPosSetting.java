package org.firstinspires.ftc.teamcode.limelight;

import static org.firstinspires.ftc.teamcode.Util.RobotPosition.modifyRobotCoordinates;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;


public class LimelightPosSetting {

    public static void positionUpdate(Limelight3A limelight, MecanumDrive drive){
        double headingOffset = 0;     // Adjust this after testing
        boolean invertHeading = false; // Set true if direction is flipped

        double smooth = 0.2;           // smoothing factor 0.1–0.3 should be fine
        double fusionWeight = 0.1;    // how much vision corrects odometry

        int crazyjump = 50;           // restrain this more if it still gives numbers

        // Filter sates
        double filteredX = 0;
        double filteredY = 0;

        drive.localizer.update();

        // Heading fix
        double headingDegrees = Math.toDegrees(drive.localizer.getPose().heading.toDouble());

        if (invertHeading) {
            headingDegrees = -headingDegrees; //TODO: make sure this is correct by seeing is it move CCW or CW
        }

        double correctedHeading = headingDegrees + headingOffset;
        limelight.updateRobotOrientation(correctedHeading);

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D pose = null;

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
            int tagCount = tags.size();

            // MT2
            if (tagCount >= 2 && result.getBotpose_MT2() != null) {
                pose = result.getBotpose_MT2();
            }
            // MT1
            else if (result.getBotpose() != null) {
                pose = result.getBotpose();
            }

            if (pose != null) {

                double x = pose.getPosition().x;
                double y = pose.getPosition().y;

                // Convert meters to inches
                double xIn = x * 39.37;
                double yIn = y * 39.37;

                // reject crazy jumps
                if (Math.abs(xIn) < crazyjump && Math.abs(yIn) < crazyjump) {

                    // Smoothing
                    filteredX = smooth * xIn + (1 - smooth) * filteredX;
                    filteredY = smooth * yIn + (1 - smooth) * filteredY;

                    // Fusion
                    Pose2d currentPose = drive.localizer.getPose();

                    double newX = currentPose.position.x * (1 - fusionWeight)
                            + filteredX * fusionWeight;

                    double newY = currentPose.position.y * (1 - fusionWeight)
                            + filteredY * fusionWeight;

                    drive.localizer.setPose(new Pose2d(
                            newX,
                            newY,
                            currentPose.heading.toDouble() // keep odometry heading
                    ));

                }
            }
        }
    }

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

                modifyRobotCoordinates(x, y, 0.0, 0.0, 0.0, yawRadians);
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

                        //drive.localizer.setPose(new Pose2d(new Vector2d(x,y),yaw));
                        modifyRobotCoordinates(x, y, 0.0, 0.0, 0.0, yaw);

                        break;
                    }
                }
            }
        }
    }
}
