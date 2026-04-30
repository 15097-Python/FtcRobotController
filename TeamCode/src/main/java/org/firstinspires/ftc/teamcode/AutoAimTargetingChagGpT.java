package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.constants.FIELD.mtoin;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="autoaimtestingAI")
@Config
public class AutoAimTargetingChagGpT extends LinearOpMode {

    // Gain for the proportional heading controller
    public static double HEADING_GAIN = 2.5;
    // Deadzone to prevent 180-degree flipping when crossing the target center
    public static double DEADZONE_INCHES = 8.0;

    private double lastTargetAngle = 0;
    public static int blue = -1;

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        waitForStart();

        while(opModeIsActive()) {
            drive.localizer.update();
            Pose2d currentPose = drive.localizer.getPose();

            // Target position (temporarily 0,0)
            double targetXInches = -1.8 * mtoin * blue;
            double targetYInches = -1.8 * mtoin;

            double dx = targetXInches - currentPose.position.x;
            double dy = targetYInches - currentPose.position.y;
            double distance = Math.hypot(dx, dy);

            // Update target angle only if outside the deadzone.
            // This prevents the "snap" rotation when passing directly over the target.
            if (distance > DEADZONE_INCHES) {
                lastTargetAngle = Math.atan2(dy, dx) - Math.PI/2;
            }
            double targetAngle = lastTargetAngle;

            // Manual translation from Gamepad 1 (Field Centric)
            double forward = -gamepad1.left_stick_y; 
            double strafe = -gamepad1.left_stick_x;  
            Vector2d fieldFrameInput = new Vector2d(forward, strafe);
            Vector2d robotFrameInput = currentPose.heading.inverse().times(fieldFrameInput);

            double rx;
            if (gamepad2.b) {
                // Non-blocking proportional heading control
                double headingError = targetAngle - currentPose.heading.toDouble();
                
                // Normalize error to [-PI, PI]
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;
                
                rx = headingError * HEADING_GAIN;
            } else {
                rx = -gamepad1.right_stick_x;
            }

            // Apply powers. This will remain active and fight back if pushed.
            drive.setDrivePowers(new PoseVelocity2d(robotFrameInput, rx));

            telemetry.addData("Target X (in)", targetXInches);
            telemetry.addData("Target Y (in)", targetYInches);
            telemetry.addData("Distance", distance);
            telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngle));
            telemetry.addData("Current Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Auto Aim Active (GP2 B)", gamepad2.b);
            telemetry.update();

            if (isStopRequested()) break;
        }
    }
}
