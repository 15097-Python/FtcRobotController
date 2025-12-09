package org.firstinspires.ftc.teamcode.Util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Detects alliance color based on Limelight AprilTag detection.
 * Tag 24 → RED  |  Tag 20 → BLUE
 * Detection happens in pipeline 3.
 */
public class AllianceDetector {

    private static final String LIMELIGHT_TABLE_NAME = "limelight";

    /**
     * Detect alliance by reading the AprilTag ID ("tid") while pipeline 3 is active.
     * @return Detected alliance or RED if unknown.
     */
    public static AllianceEnumerations detectAllianceFromTag(Telemetry telemetry) {
        try {
            NetworkTable limelight = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);

            // Make sure we’re on the detection pipeline (3)
            limelight.getEntry("pipeline").setNumber(3);

            // Read AprilTag ID the Limelight currently sees
            double tagID = limelight.getEntry("tid").getDouble(-1);

            AllianceEnumerations alliance;

            if (tagID == 20) {
                alliance = AllianceEnumerations.BLUE;
            } else if (tagID == 24) {
                alliance = AllianceEnumerations.RED;
            } else {
                alliance = AllianceEnumerations.RED; // default/fallback
            }

            if (telemetry != null) {
                telemetry.addData("Detected Tag ID", tagID);
                telemetry.addData("Detected Alliance", alliance);
            }

            return alliance;

        } catch (Exception e) {
            if (telemetry != null)
                telemetry.addData("Alliance", "Error reading AprilTag — defaulting to RED");
            return AllianceEnumerations.RED;
        }
    }

    /** Set Limelight to a given pipeline (1 = RED, 2 = BLUE, 3 = Detection). */
    public static void setLimelightPipeline(int pipelineNumber, Telemetry telemetry) {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable(LIMELIGHT_TABLE_NAME);
        limelight.getEntry("pipeline").setNumber(pipelineNumber);
        if (telemetry != null)
            telemetry.addData("Pipeline Set To", pipelineNumber);
    }

    /** Switch to the proper alliance pipeline after detection. */
    public static void switchToAlliancePipeline(AllianceEnumerations alliance, Telemetry telemetry) {
        int pipeline = (alliance == AllianceEnumerations.BLUE) ? 2 : 1;
        setLimelightPipeline(pipeline, telemetry);
    }
}
