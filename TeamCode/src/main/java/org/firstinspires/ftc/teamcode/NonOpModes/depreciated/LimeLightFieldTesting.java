package org.firstinspires.ftc.teamcode.NonOpModes.depreciated;

import static org.firstinspires.ftc.teamcode.limelight.LimelightPosSetting.*;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
                    drive.localizer.setPose(new Pose2d(new Vector2d(xIn, yIn), yawRadians));

                    // Format string: inserts a value into the text with specific formatting (here, a float rounded to 1 decimal place)
                    telemetry.addLine()
                            .addData("MT2 raw (m)", "x=%.2f y=%.2f", x, y)
                            .addData("Converted (in)", "x=%.2f y=%.2f", xIn, yIn)
                            .addData("Yaw", "deg=%.1f rad=%.3f", yawDegrees, yawRadians)
                            .addData("headingDegrees", "deg=%.1f", headingDegrees) //TODO check if this increases or decreases with CW rotation
                            .addData("Drive pose", "x=%.2f y=%.2f h=%.2f",
                                    drive.localizer.getPose().position.x,
                                    drive.localizer.getPose().position.y,
                                    Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
                }
            }
            telemetry.update();
        }

        limelight.stop();

    } 
}

