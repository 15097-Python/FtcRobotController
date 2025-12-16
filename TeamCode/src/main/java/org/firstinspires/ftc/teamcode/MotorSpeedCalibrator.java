package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Util.constants.RobotStats.ENCODER_TICKS_PER_OUTPUT_REV;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Speed Calibrator", group = "Diagnostics")
public class MotorSpeedCalibrator extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "LauncherFL");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("=== Motor Speed Calibration ===");
        telemetry.addLine("1. Run motor at 100% power");
        telemetry.addLine("2. Let it stabilize for 2-3 seconds");
        telemetry.addLine("3. The 'Measured RPM' is your MaxRPM constant");
        telemetry.update();

        waitForStart();

        motor.setPower(1.0); // Full power

        while (opModeIsActive()) {
            double ticksPerSecond = motor.getVelocity();
            double measuredRPM = (ticksPerSecond / ENCODER_TICKS_PER_OUTPUT_REV) * 60.0;

            telemetry.addData("Measured RPM", "%.1f", measuredRPM);
            telemetry.addData("Ticks/sec", "%.0f", ticksPerSecond);
            telemetry.addLine("\nUse this value for MaxRPM constant");
            telemetry.update();

            sleep(100);
        }
        motor.setPower(0.0);
    }
}