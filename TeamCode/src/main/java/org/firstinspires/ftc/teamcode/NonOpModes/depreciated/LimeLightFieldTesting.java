/*package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="LimeLightFieldTesting", group="limelight")


public class LimeLightFieldTesting extends LinearOpMode {

    @Override

    public void runOpMode() {

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");// Initializes the limelights
        limelight.setPollRateHz(90);
        limelight.pipelineSwitch(0);
        limelight.start();

        Pose2d startPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        waitForStart();

        while (opModeIsActive()) {
            drive.localizer.update();

            double headingDegrees = Math.toDegrees(drive.localizer.getPose().heading.toDouble());
            limelight.updateRobotOrientation(headingDegrees);
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D robotPoseMT2 = result.getBotpose_MT2();

                if (robotPoseMT2 != null) {
                    double x = robotPoseMT2.getPosition().x;
                    double y = robotPoseMT2.getPosition().y;
                    double yawDegrees = robotPoseMT2.getOrientation().getYaw();
                    double xIn = 39.37 * x;
                    double yIn = 39.37 * y;
                    double yawRadians = Math.toRadians(-yawDegrees);

                    // Format string: inserts a value into the text with specific formatting (here, a float rounded to 1 decimal place)
                    telemetry.addData("MT2 raw (m)", "x=%.2f y=%.2f", x, y);
                    telemetry.addData("Converted (in)", "x=%.2f y=%.2f", xIn, yIn);
                    telemetry.addData("Yaw", "deg=%.1f rad=%.3f", yawDegrees, yawRadians);
                    telemetry.addData("headingDegrees", "deg=%.1f", headingDegrees); //TODO check if this increases or decreases with CW rotation
                    telemetry.addData("Drive pose", "x=%.2f y=%.2f h=%.2f",
                                    drive.localizer.getPose().position.x,
                                    drive.localizer.getPose().position.y,
                                    Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
                }
            }
            telemetry.update();
        }

        limelight.stop();

    } 
}*/

package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Autonomous(name="LimeLightFieldTesting", group="limelight")

public class LimeLightFieldTesting extends LinearOpMode {

    //Tuning Constants
    double headingOffset = 0;     // Adjust this after testing
    boolean invertHeading = false; // Set true if direction is flipped

    double alpha = 0.2;           // smoothing factor (0.1–0.3 good)
    double fusionWeight = 0.1;    // how much vision corrects odometry

    int crazyjump = 500;

    // Filter sates
    double filteredX = 0;
    double filteredY = 0;

    @Override
    public void runOpMode() {

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(90);
        limelight.pipelineSwitch(0);
        limelight.start();

        Pose2d startPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        while (opModeIsActive()) {

            drive.localizer.update();

            // Heading fix
            double headingDegrees = Math.toDegrees(drive.localizer.getPose().heading.toDouble());

            if (invertHeading) {
                headingDegrees = -headingDegrees;
            }

            double correctedHeading = headingDegrees + headingOffset;
            limelight.updateRobotOrientation(correctedHeading);

            LLResult result = limelight.getLatestResult();

            boolean usingVision = false;

            if (result != null && result.isValid()) {

                Pose3D pose = null;

                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                int tagCount = tags.size();

                // ====== USE MT2 ONLY IF MULTI-TAG ======
                if (tagCount >= 2 && result.getBotpose_MT2() != null) {
                    pose = result.getBotpose_MT2();
                }
                // ====== FALLBACK TO MT1 ======
                else if (result.getBotpose() != null) {
                    pose = result.getBotpose();
                }

                if (pose != null) {

                    double x = pose.getPosition().x;
                    double y = pose.getPosition().y;
                    double yawDegrees = pose.getOrientation().getYaw();

                    // Convert meters → inches
                    double xIn = x * 39.37;
                    double yIn = y * 39.37;

                    // reject crazy jumps
                    if (Math.abs(xIn) < crazyjump && Math.abs(yIn) < crazyjump) {

                        // Smoothing
                        filteredX = alpha * xIn + (1 - alpha) * filteredX;
                        filteredY = alpha * yIn + (1 - alpha) * filteredY;

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

                        usingVision = true;

                        // telemetry
                        telemetry.addData("Vision Raw (in)", "x=%.2f y=%.2f", xIn, yIn);
                        telemetry.addData("Vision Filtered", "x=%.2f y=%.2f", filteredX, filteredY);
                        telemetry.addData("Yaw", "deg=%.1f", yawDegrees);
                    }

                    telemetry.addData("Tag Count", tagCount);
                }
            }

            // drive telemetry
            Pose2d pose = drive.localizer.getPose();

            telemetry.addData("Using Vision", usingVision);
            telemetry.addData("Heading (deg)", correctedHeading);
            telemetry.addData("Drive Pose", "x=%.2f y=%.2f h=%.2f",
                    pose.position.x,
                    pose.position.y,
                    Math.toDegrees(pose.heading.toDouble()));

            telemetry.update();
        }

        limelight.stop();
    }
}

