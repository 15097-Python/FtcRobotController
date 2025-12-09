/*
package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.getRobotCoordinates;
import static org.firstinspires.ftc.teamcode.Util.RobotPositionCrosby.modifyRobotCoordinates;
import static org.firstinspires.ftc.teamcode.Util.constants.Conversions.InToM;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;


public class GoBuildaPinpointLocalizer implements Localizer {
    private GoBildaPinpointDriver odometry;

    public void GoBildaLocalizer() {
        // "pinpoint" must match the name in your robot configuration
        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Optionally reset pose / IMU if starting fresh:
        odometry.resetPosAndIMU();
    }

    @Override
    public void setPose(Pose2d pose) {

    }

    @Override
    public Pose2d getPose() {


        double xIn  = odometry.getPosX(DistanceUnit.INCH);
        double yIn  = odometry.getPosY(DistanceUnit.INCH);
        double headingRad = odometry.getHeading(AngleUnit.RADIANS);
        double[] robotPos = getRobotCoordinates();
        modifyRobotCoordinates(xIn*InToM,yIn*InToM,robotPos[2],robotPos[3],robotPos[4],robotPos[5]);
        return new Pose2d(xIn, yIn, headingRad);
    }

    @Override
    public PoseVelocity2d update() {
        return null;
    }


    public void setPose() {
        double[] robotPos = getRobotCoordinates();
        Pose2D robot2DPos = new Pose2D(DistanceUnit.METER,(robotPos[0]*1000), (robotPos[1]*1000),AngleUnit.RADIANS, robotPos[5]);
        odometry.setPosition(robot2DPos);
    }


    public Pose2d getPoseVelocity() {
        double xVelINCHPerSec = odometry.getVelX(DistanceUnit.INCH);
        double yVelINCHPerSec = odometry.getVelY(DistanceUnit.INCH);
        double headingRad = odometry.getHeading(AngleUnit.RADIANS); // raw heading in radians
        double headingVelRadPerSec = odometry.getHeadingVelocity(AngleUnit.RADIANS.getUnnormalized());

        Pose2D vel = new Pose2D(DistanceUnit.INCH, xVelINCHPerSec, yVelINCHPerSec, AngleUnit.RADIANS, headingVelRadPerSec);

        return vel;
    }

}
*/