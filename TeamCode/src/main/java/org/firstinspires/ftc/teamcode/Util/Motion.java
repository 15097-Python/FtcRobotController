package org.firstinspires.ftc.teamcode.Util;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


public class Motion {


    private LinearOpMode opMode;
    private GoBildaPinpointDriver odom;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;


    // TODO: tune these value
    public double kP_drive = 1.1; // meters -> power
    public double kP_turn = 2.0; // radians -> power
    public double maxDrivePower = 0.6;
    public double maxTurnPower = 0.35;


    public void MotionController(LinearOpMode opMode,
                                 GoBildaPinpointDriver odom,
                                 DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR) {
        this.opMode = opMode;
        this.odom = odom;
        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;
    }

    public Motion(LinearOpMode opMode, GoBildaPinpointDriver odom, DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.opMode = opMode;
        this.odom = odom;
        FL = fl;
        FR = fr;
        BL = bl;
        BR = br;
    }


    // ================= DRIVE =================
    public boolean driveTo(double targetX, double targetY,
                           double toleranceMeters, double timeoutSec) {


        double start = opMode.getRuntime();


        while (opMode.opModeIsActive()) {
            if (opMode.getRuntime() - start > timeoutSec) return false;


            odom.update();
            double x = odom.getPosX(DistanceUnit.METER);
            double y = odom.getPosY(DistanceUnit.METER);


            double dx = targetX - x;
            double dy = targetY - y;
            double distance = Math.hypot(dx, dy);


            if (distance < toleranceMeters) break;


            double power = clamp(distance * kP_drive, 0, maxDrivePower);


            double vx = (dx / distance) * power;
            double vy = (dy / distance) * power;


            applyFieldCentric(vx, vy);
            opMode.sleep(10);
        }


        stop();
        return true;
    }


    //TURN
    public boolean turnTo(double targetHeadingRad,
                          double toleranceRad, double timeoutSec) {


        double start = opMode.getRuntime();


        while (opMode.opModeIsActive()) {
            if (opMode.getRuntime() - start > timeoutSec) return false;


            odom.update();
            double current = odom.getHeading(AngleUnit.RADIANS);
            double error = angleWrap(targetHeadingRad - current);


            if (Math.abs(error) < toleranceRad) break;


            double turn = clamp(error * kP_turn,
                    -maxTurnPower, maxTurnPower);


            applyRobotCentric(0, 0, turn);
            opMode.sleep(10);
        }


        stop();
        return true;
    }


    // MATH
    private void applyFieldCentric(double vx, double vy) {
        double heading = odom.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);


        double rx = vx * cos - vy * sin;
        double ry = vx * sin + vy * cos;


        applyRobotCentric(rx, ry, 0);
    }


    private void applyRobotCentric(double vx, double vy, double omega) {
        double fl = vy + vx + omega;
        double fr = vy - vx - omega;
        double bl = vy - vx + omega;
        double br = vy + vx - omega;


        double max = Math.max(1.0,
                Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));


        FL.setPower(fl / max);
        FR.setPower(fr / max);
        BL.setPower(bl / max);
        BR.setPower(br / max);
    }


    public void stop() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }


    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }


    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}