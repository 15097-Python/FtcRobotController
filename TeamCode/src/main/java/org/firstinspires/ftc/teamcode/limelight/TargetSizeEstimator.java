package org.firstinspires.ftc.teamcode.limelight;

public class TargetSizeEstimator {

    /**
     * Estimates the real-world width of an object using pixel width, camera FOV, resolution, and distance.
     *
     * @param pixelWidth   Width of the target blob in pixels.
     * @param fovDegrees   Horizontal Field of View of the camera in degrees.
     * @param resolutionPx Horizontal resolution of the camera in pixels.
     * @param distanceM    Distance from camera to target in meters.
     * @return Estimated real-world width in meters.
     */
    public static double estimateRealWidth(int pixelWidth, double fovDegrees, int resolutionPx, double distanceM) {
        // Calculate angle per pixel
        double anglePerPixel = fovDegrees / resolutionPx;

        // Calculate total angle covered by the target
        double totalAngleDegrees = pixelWidth * anglePerPixel;

        // Use trigonometry to calculate real-world width
        double halfAngleRadians = Math.toRadians(totalAngleDegrees / 2.0);
        double realWidth = 2 * distanceM * Math.tan(halfAngleRadians);

        return realWidth;
    }
}

