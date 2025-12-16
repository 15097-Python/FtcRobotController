package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Util.Enum.Balls;
import org.firstinspires.ftc.teamcode.Util.RPMEstimator;
import org.firstinspires.ftc.teamcode.NonOpModes.colorsensing.ColorSensingFunctions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import static org.firstinspires.ftc.teamcode.Util.constants.RobotStats.*;

/** TODO: implement a way to tell where to hit the gate and where to do so
 *        implement a way to park near end of auto
 *        test current op mode and check for bugs or errors
 *        implement a way to check pattern status in gate */

@Autonomous(name="AutoOdomBased", group="Auto")
public class AutoOdomBased extends LinearOpMode {

    // hardware
    private Limelight3A limelight;
    private GoBildaPinpointDriver odomhub;
    private DcMotorEx launcherMotor;
    private DcMotor Scooper;
    private Servo drumServo;
    private Servo firingPinServo;
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private DcMotor BR, BL, FR, FL;

    // states
    private enum AutoState {
        INITIALIZE,
        DRIVE_TO_SHOOT_POSITION,
        DETECT_SCORING_TAG,
        CALCULATE_AND_SPIN_UP,
        FIND_CORRECT_BALL,
        AIM_AND_FIRE,
        PARK,
        COMPLETE
    }

    // constants
    private static final int PIPELINE_SCORING = 0;
    private static final int PIPELINE_PATTERN = 1;
    private static final int PIPELINE_BALL = 4;
    private static final double LIMELIGHT_MIN_RANGE_METERS = 0.9144; // 3 feet
    private static final double MAX_SHOOTING_RANGE = 3.7; // meters

    private static final double[] DRUM_FIRE_POSITIONS = {0.1, 0.42, 0.76};
    private static final double[] DRUM_LOAD_POSITIONS = {0.27, 0.6, 0.92};
    private static final double FIRING_PIN_NULL = 0.98;
    private static final double FIRING_PIN_FIRE = FIRING_PIN_NULL - 0.32;

    // I'm storing it
    private AutoState currentState = AutoState.INITIALIZE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private double targetDistanceMeters = 0;
    private int targetBallSlot = 0;
    private final Balls targetBallColor = Balls.green;

    @Override
    public void runOpMode() {
        initializeHardware();

        // ===== ALLIANCE SELECTION (INTEGRATED) =====
        telemetry.addLine("SELECT ALLIANCE");
        telemetry.addLine("Press X for BLUE, B for RED");
        telemetry.addLine("Current: RED (default)");
        telemetry.update();

        boolean isBlueSelected = false;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                isBlueSelected = true;
                telemetry.addLine("BLUE alliance selected");
                telemetry.update();
                sleep(200);
            }
            if (gamepad1.b) {
                isBlueSelected = false;
                telemetry.addLine("RED alliance selected");
                telemetry.update();
                sleep(200);
            }
        }

        final boolean isRedAlliance = !isBlueSelected;

        telemetry.clearAll();
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Target Ball", targetBallColor);
        telemetry.addData("Status", "Auto Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            executeStateMachine(isRedAlliance); // Pass alliance to state machine
        }
    }

    private void initializeHardware() {
        // Odometry
        odomhub = hardwareMap.get(GoBildaPinpointDriver.class, "odomhub");
        odomhub.initialize();
        odomhub.resetPosAndIMU();

        // Drive motors
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // Launcher
        launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherFL");
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Scooper
        Scooper = hardwareMap.get(DcMotorEx.class, "Scooper");

        // Servos
        drumServo = hardwareMap.get(Servo.class, "DrumServo");
        firingPinServo = hardwareMap.get(Servo.class, "FiringPinServo");

        // Color sensors
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");

        // Initial positions
        drumServo.setPosition(DRUM_LOAD_POSITIONS[0]);
        firingPinServo.setPosition(FIRING_PIN_NULL);
    }

    private void executeStateMachine(boolean isRedAlliance) {
        switch (currentState) {
            case INITIALIZE:
                telemetry.addData("State", "INITIALIZE");
                telemetry.update();
                sleep(500);
                currentState = AutoState.DRIVE_TO_SHOOT_POSITION;
                break;

            case DRIVE_TO_SHOOT_POSITION:
                telemetry.addData("State", "DRIVE TO SHOOT POSITION");
                driveToPosition(2.0, 1.5, 0.45); // {check} if this is right
                turnToHeading(Math.toRadians(45));
                currentState = AutoState.DETECT_SCORING_TAG;
                break;

            case DETECT_SCORING_TAG:
                telemetry.addData("State", "DETECT SCORING TAG");
                limelight.pipelineSwitch(PIPELINE_SCORING);
                sleep(500);

                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid() && result.getFiducialResults() != null) {
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        int tagId = tag.getFiducialId();

                        boolean isTargetTag = (isRedAlliance && tagId == 1) ||
                                (!isRedAlliance && tagId == 2);

                        if (isTargetTag) {
                            Pose3D botpose = tag.getRobotPoseFieldSpace();
                            targetDistanceMeters = calculateDistanceToTarget(botpose);

                            telemetry.addData("Target Tag", tagId);
                            telemetry.addData("Distance", "%.2f m", targetDistanceMeters);

                            if (targetDistanceMeters > LIMELIGHT_MIN_RANGE_METERS &&
                                    targetDistanceMeters <= MAX_SHOOTING_RANGE) {
                                currentState = AutoState.CALCULATE_AND_SPIN_UP;
                                return;
                            }
                        }
                    }
                    telemetry.addData("ERROR", "Target tag not found");
                } else {
                    telemetry.addData("ERROR", "No AprilTags detected");
                }
                telemetry.update();
                break;

            case CALCULATE_AND_SPIN_UP:
                telemetry.addData("State", "CALCULATE AND SPIN UP");
                try {
                    double requiredRPM = RPMEstimator.calculateRequiredRPM(targetDistanceMeters);
                    double ticksPerSec = RPMEstimator.wheelRPMToEncoderVelocity(requiredRPM);
                    double commandRadPerSec = (ticksPerSec / ENCODER_TICKS_PER_OUTPUT_REV) * 2 * Math.PI;

                    telemetry.addData("Required RPM", "%.0f", requiredRPM);
                    telemetry.addData("Command (rad/s)", "%.2f", commandRadPerSec);

                    launcherMotor.setVelocity(commandRadPerSec, AngleUnit.RADIANS);

                    stateTimer.reset();
                    while (opModeIsActive() && stateTimer.milliseconds() < 3000) {
                        if (isAtTargetRPM(requiredRPM)) break;
                        telemetry.addData("Spinning up ", getCurrentRPM());
                        telemetry.update();
                    }

                    currentState = AutoState.FIND_CORRECT_BALL;
                } catch (Exception e) {
                    telemetry.addData("ERROR", e.getMessage());
                    telemetry.update();
                    sleep(2000);
                }
                break;

            case FIND_CORRECT_BALL:
                telemetry.addData("State", "FIND CORRECT BALL");
                limelight.pipelineSwitch(PIPELINE_BALL);
                sleep(500);

                boolean found = false;
                for (int slot = 0; slot < 3; slot++) {
                    drumServo.setPosition(DRUM_LOAD_POSITIONS[slot]);
                    sleep(300);

                    Balls ball = ColorSensingFunctions.colorDetection(colorSensor1, colorSensor2);
                    telemetry.addData("Slot " + slot, ball);

                    if (ball == targetBallColor) {
                        targetBallSlot = slot;
                        found = true;
                        break;
                    }
                }

                if (found) {
                    currentState = AutoState.AIM_AND_FIRE;
                } else {
                    telemetry.addData("ERROR", "Ball not found");
                }
                telemetry.update();
                break;

            case AIM_AND_FIRE:
                telemetry.addData("State", "AIM AND FIRE");
                drumServo.setPosition(DRUM_FIRE_POSITIONS[targetBallSlot]);
                sleep(500);

                firingPinServo.setPosition(FIRING_PIN_FIRE);
                telemetry.addData("Firing", "YES!");
                telemetry.update();
                sleep(200);

                firingPinServo.setPosition(FIRING_PIN_NULL);
                currentState = AutoState.PARK;
                break;

            case PARK:
                telemetry.addData("State", "PARK");
                driveToPosition(0.5, 0.5, 0.3);
                launcherMotor.setVelocity(0);
                setMotorPowers(0, 0, 0, 0);
                telemetry.addData("Autonomous", "COMPLETE");
                telemetry.update();
                currentState = AutoState.COMPLETE;
                break;

            case COMPLETE:
                requestOpModeStop();
                break;
        }
        telemetry.update();
    }

    private void driveToPosition(double targetX, double targetY, double power) {
        final double POSITION_TOLERANCE = 0.1;
        while (opModeIsActive()) {
            odomhub.update();
            double currentX = odomhub.getPosX(DistanceUnit.METER);
            double currentY = odomhub.getPosY(DistanceUnit.METER);

            double dx = targetX - currentX;
            double dy = targetY - currentY;
            double distance = Math.sqrt(dx*dx + dy*dy);

            if (distance < POSITION_TOLERANCE) break;

            double vx = (dx / distance) * power;
            double vy = (dy / distance) * power;

            double heading = odomhub.getHeading(AngleUnit.RADIANS);
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double vx_rotated = vx * cos - vy * sin;
            double vy_rotated = vx * sin + vy * cos;

            double fl = vy_rotated + vx_rotated;
            double bl = vy_rotated - vx_rotated;
            double br = vy_rotated - vx_rotated;
            double fr = vy_rotated + vx_rotated;

            double max = Math.max(1.0, Math.abs(fl));
            max = Math.max(max, Math.abs(bl));
            max = Math.max(max, Math.abs(br));
            max = Math.max(max, Math.abs(fr));

            setMotorPowers(fl/max, bl/max, br/max, fr/max);
        }
        setMotorPowers(0, 0, 0, 0);
    }

    private void turnToHeading(double targetHeading) {
        final double HEADING_TOLERANCE = Math.toRadians(5);
        while (opModeIsActive()) {
            odomhub.update();
            double currentHeading = odomhub.getHeading(AngleUnit.RADIANS);
            double error = targetHeading - currentHeading;

            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;

            if (Math.abs(error) < HEADING_TOLERANCE) break;

            double turnPower = error * 0.5;
            turnPower = Math.max(-0.3, Math.min(0.3, turnPower));

            setMotorPowers(-turnPower, -turnPower, turnPower, turnPower);
        }
        setMotorPowers(0, 0, 0, 0);
    }

    private void setMotorPowers(double fl, double bl, double br, double fr) {
        FL.setPower(fl);
        BL.setPower(bl);
        BR.setPower(br);
        FR.setPower(fr);
    }

    private LLResult getLimelightResult() {   //if limelight doesn't have enough time for processing use this
        for (int i = 0; i < 50 && opModeIsActive(); i++) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) return result;
            sleep(100);
        }
        return null;
    }

    private double calculateDistanceToTarget(Pose3D botpose) {
        if (botpose == null) return 999;
        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;
        return Math.sqrt(x * x + y * y);
    }

    private double getCurrentRPM() {
        double ticksPerSecond = launcherMotor.getVelocity();
        return (ticksPerSecond / ENCODER_TICKS_PER_OUTPUT_REV) * 60.0;
    }

    private boolean isAtTargetRPM(double targetRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) <= 5.0;
    }
}